#pragma once
#include "utils/drivers/telemetry_protocol.hpp"
#include "utils/logger.hpp"
#include "utils/scheduler/scheduler.hpp"
#include <utility>
#include <boost/asio.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace awakening {

class SerialDriver {
public:
    struct Params {
        unsigned int baud_rate = 115200, char_size = 8;
        std::string device_name;
        size_t read_buf_size = 4096;
        int telemetry_gimbal_id = -1, telemetry_shoot_id = -1;
        int frame_timeout_ms = 1000, idle_timeout_ms = 3000, reconnect_ms = 500;
        void load(const YAML::Node& c) {
            device_name = c["device_name"].as<std::string>();
            baud_rate = c["baud_rate"].as<unsigned int>(115200);
            char_size = c["char_size"].as<unsigned int>(8);
            read_buf_size = c["read_buf_size"].as<size_t>(4096);
            telemetry_gimbal_id = c["telemetry_gimbal_id"].as<int>(-1);
            telemetry_shoot_id = c["telemetry_shoot_id"].as<int>(-1);
            frame_timeout_ms = c["frame_timeout_ms"].as<int>(1000);
            idle_timeout_ms = c["idle_timeout_ms"].as<int>(3000);
            reconnect_ms = c["reconnect_ms"].as<int>(500);
            if (device_name.empty() || baud_rate == 0 || char_size != 8
                || read_buf_size == 0 || read_buf_size > 65536
                || frame_timeout_ms <= 0 || idle_timeout_ms <= 0 || reconnect_ms <= 0)
                throw std::invalid_argument("invalid telemetry serial parameters (8-bit data required)");
            const bool unknown = telemetry_gimbal_id == -1 && telemetry_shoot_id == -1;
            const bool valid = telemetry_gimbal_id > 0 && telemetry_gimbal_id <= 32767
                && telemetry_shoot_id > 0 && telemetry_shoot_id <= 32767
                && telemetry_gimbal_id != telemetry_shoot_id;
            if (!unknown && !valid)
                throw std::invalid_argument("telemetry IDs must be distinct positive int16 values, or both -1");
        }
    } params_;

    struct Statistics { uint64_t frames, invalid, connections, timeouts, framing_errors; };
    Statistics statistics() const {
        return {frames_.load(), invalid_.load(), connections_.load(),
                timeouts_.load(), framing_errors_.load()};
    }
    SerialDriver(const YAML::Node& config, Scheduler& scheduler)
        : scheduler_(scheduler), port_(io_), timer_(io_) {
        params_.load(config);
    }
    ~SerialDriver() { stop(); }
    SerialDriver(const SerialDriver&) = delete;
    SerialDriver& operator=(const SerialDriver&) = delete;

    template<class Tag>
    void start(std::string source_name) {
        using IO = IOPair<Tag, std::vector<uint8_t>>;
        std::lock_guard<std::mutex> lock(lifecycle_);
        if (thread_.joinable()) return;
        source_snapshot_id_ = scheduler_.register_source<IO>(source_name);
        // 类型擦除的投递通道：同济有效完整帧或其它协议原始字节交给业务侧 receive_serial。
        publish_ = [this](std::vector<uint8_t> bytes) {
            // 只在 Scheduler::run() 之后投递，避免与 main 线程的 build() 发生竞争。
            if (bytes.empty() || !scheduler_.is_running()) return;
            scheduler_.runtime_push_source<IO>(
                source_snapshot_id_,
                [payload = std::move(bytes)]() mutable {
                    return std::make_tuple(std::optional<std::vector<uint8_t>>(std::move(payload)));
                }
            );
        };
        active_ = params_;
        read_buf_.resize(active_.read_buf_size);
        stream_ = TelemetryStream(std::chrono::milliseconds(active_.frame_timeout_ms));
        stop_requested_ = false;
        thread_ = std::thread([this] { run(); });
    }
    void stop() {
        std::lock_guard<std::mutex> lock(lifecycle_);
        stop_requested_ = true;
        connected_.store(false, std::memory_order_release);
        retry_cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }
    // 上位机 -> 下位机。data 必须是一帧完整报文（见 utils/drivers/command_protocol.hpp）：
    // 固件每次 USB 回调只消费首部声明的字节数，多余的会被丢弃，因此不能把多帧拼在一次调用里。
    // 注意：独立 write 不能保证 USB 回调边界，固件仍需按实际接收长度处理拆包/粘包。
    bool write(const std::vector<uint8_t>& data) {
        if (!connected_.load(std::memory_order_acquire)) return false;
        const uint64_t gen = generation_.load(std::memory_order_acquire);
        boost::asio::post(io_, [this, data, gen]() mutable {
            if (gen != generation_.load(std::memory_order_acquire)) return; // 已重连，丢弃旧连接上的包
            if (write_queue_.size() >= kMaxQueuedFrames) return;            // 背压：串口堵塞时丢弃新帧
            const bool idle = write_queue_.empty();
            write_queue_.push_back(std::move(data));
            if (idle) do_write(gen);
        });
        return true;
    }
private:
    // 只有配置了同济状态回传 ID 才启用严格组帧解析；未配置的用途
    // （例如 radar_detect 的裁判/图传串口）按纯字节流处理，不做帧同步判定。
    bool telemetry_enabled() const noexcept {
        return active_.telemetry_gimbal_id > 0 && active_.telemetry_shoot_id > 0;
    }
    bool open() {
        boost::system::error_code ec;
        port_.open(active_.device_name, ec);
        if (!ec) port_.set_option(boost::asio::serial_port_base::baud_rate(active_.baud_rate), ec);
        if (!ec) port_.set_option(boost::asio::serial_port_base::character_size(8), ec);
        if (!ec) port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), ec);
        if (!ec) port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), ec);
        if (!ec) port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), ec);
        if (ec) {
            AWAKENING_WARN("Telemetry serial open/configure failed: {}", ec.message());
            boost::system::error_code ignored;
            if (port_.is_open()) port_.close(ignored);
            return false;
        }
        ++connections_;
        AWAKENING_INFO(
            "Serial connected: {} (telemetry RX id={}/{} + command TX)",
            active_.device_name,
            active_.telemetry_gimbal_id,
            active_.telemetry_shoot_id
        );
        return true;
    }
    void end_connection() {
        if (ending_) return;
        ending_ = true;
        connected_.store(false, std::memory_order_release);
        boost::system::error_code ignored;
        timer_.cancel(ignored);
        if (port_.is_open()) {
            port_.cancel(ignored);
            port_.close(ignored);
        }
        // 注意：这里不能清 write_queue_。可能还有 async_write 在途，其缓冲区指向队首元素；
        // 队列统一在下一连接建立、io_.run() 返回之后清理（见 run()）。
    }
    // 串行发送写队列。gen 用于丢弃上一个连接残留在 io_context 里的任务。
    void do_write(uint64_t gen) {
        if (write_queue_.empty()) return;
        boost::asio::async_write(
            port_,
            boost::asio::buffer(write_queue_.front()),
            [this, gen](const boost::system::error_code& ec, size_t) {
                if (gen != generation_.load(std::memory_order_acquire)) return;
                if (ec) {
                    if (ec != boost::asio::error::operation_aborted)
                        AWAKENING_WARN("Serial TX failed: {}", ec.message());
                    end_connection();
                    return;
                }
                write_queue_.pop_front();
                if (!write_queue_.empty()) do_write(gen);
            }
        );
    }
    void watchdog() {
        timer_.expires_after(std::chrono::milliseconds(20));
        timer_.async_wait([this](const boost::system::error_code& ec) {
            if (ec || ending_) return;
            if (stop_requested_) { end_connection(); return; }
            const auto now = TelemetryStream::Clock::now();
            if (stream_.expired(now)
                || now - last_rx_ >= std::chrono::milliseconds(active_.idle_timeout_ms)) {
                ++timeouts_;
                AWAKENING_WARN("Telemetry receive timeout; discarding partial frame and reconnecting (no subscription sent)");
                end_connection();
                return;
            }
            watchdog();
        });
    }
    void read() {
        port_.async_read_some(boost::asio::buffer(read_buf_),
            [this](const boost::system::error_code& ec, size_t size) {
                if (ending_) return;
                if (ec || stop_requested_) {
                    if (ec && ec != boost::asio::error::operation_aborted)
                        AWAKENING_WARN("Telemetry read failed: {}", ec.message());
                    end_connection();
                    return;
                }
                const auto now = TelemetryStream::Clock::now();
                if (size) {
                    last_rx_ = now;
                    // 非同济协议保持原始字节透传；同济状态在 observe 中按完整有效帧投递。
                    // 不把半包交给业务层，避免重连后业务解析器残留上一连接的数据。
                    if (!telemetry_enabled() && publish_) {
                        publish_(std::vector<uint8_t>(read_buf_.data(), read_buf_.data() + size));
                    }
                }
                if (telemetry_enabled() && !stream_.feed(read_buf_.data(), size, now,
                    [this, now](const uint8_t* p, size_t n) { observe(p, n, now); })) {
                    if (stream_.expired(now)) ++timeouts_;
                    else ++framing_errors_;
                    AWAKENING_WARN("Telemetry framing failed; reconnecting without guessing a sync position");
                    end_connection();
                    return;
                }
                read();
            });
    }
    void observe(const uint8_t* p, size_t n, TelemetryStream::Clock::time_point now) {
        ++frames_;
        TongjiTelemetry state{};
        const bool valid = TongjiTelemetry::decode(p, n, active_.telemetry_gimbal_id,
                                                   active_.telemetry_shoot_id, state);
        if (!valid) ++invalid_;
        if (valid && publish_) {
            std::vector<uint8_t> frame;
            frame.reserve(n + 2);
            frame.push_back(static_cast<uint8_t>(n & 0xff));
            frame.push_back(static_cast<uint8_t>((n >> 8) & 0xff));
            frame.insert(frame.end(), p, p + n);
            publish_(std::move(frame));
        }
        if (now - last_log_ < std::chrono::seconds(1)) return;
        last_log_ = now;
        if (valid)
            AWAKENING_INFO("Telemetry RX: yaw_deg={} pitch_deg={} roll_deg={} mode_raw={} robot_id={} bullet_speed={}",
                           state.yaw, state.pitch, state.roll, state.mode, state.robot_id, state.bullet_speed);
        else
            AWAKENING_WARN("Telemetry unparsed/rejected: check configured IDs, layout and values; no CRC exists in this protocol");
    }
    void run() {
        while (!stop_requested_) {
            try {
                if (open()) {
                    // 新连接开始：上一个 io_.run() 已返回，没有在途 handler，
                    // 此时清理写队列并推进 generation_ 是安全的（旧连接的投递任务会被丢弃）。
                    write_queue_.clear();
                    generation_.fetch_add(1, std::memory_order_acq_rel);
                    connected_.store(true, std::memory_order_release);
                    stream_.reset();
                    ending_ = false;
                    last_rx_ = TelemetryStream::Clock::now();
                    io_.restart();
                    read();
                    watchdog();
                    io_.run();
                }
            } catch (const std::exception& error) {
                AWAKENING_ERROR("Telemetry IO exception: {}", error.what());
                end_connection();
                io_.restart();
                io_.run();
            }
            std::unique_lock<std::mutex> lock(retry_mutex_);
            retry_cv_.wait_for(lock, std::chrono::milliseconds(active_.reconnect_ms),
                              [this] { return stop_requested_.load(); });
        }
    }
    Params active_;
    Scheduler& scheduler_;
    size_t source_snapshot_id_ = 0;
    std::function<void(std::vector<uint8_t>)> publish_;
    boost::asio::io_context io_;
    boost::asio::serial_port port_;
    boost::asio::steady_timer timer_;
    TelemetryStream stream_;
    std::vector<uint8_t> read_buf_;
    std::deque<std::vector<uint8_t>> write_queue_;
    static constexpr size_t kMaxQueuedFrames = 8;
    std::atomic<uint64_t> generation_{0};
    std::atomic<bool> connected_{false};
    bool ending_ = true;
    TelemetryStream::Clock::time_point last_rx_{}, last_log_{};
    std::atomic<bool> stop_requested_{true};
    std::atomic<uint64_t> frames_{0}, invalid_{0}, connections_{0}, timeouts_{0}, framing_errors_{0};
    std::mutex lifecycle_, retry_mutex_;
    std::condition_variable retry_cv_;
    std::thread thread_;
};
} // namespace awakening
