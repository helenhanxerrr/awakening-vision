#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
namespace awakening {
struct TongjiTelemetry {
    float yaw, pitch, roll, bullet_speed;
    // Firmware calls the last wire field color, but assigns uint8 robot_id to it.
    int32_t mode, robot_id;

    static uint16_t u16(const uint8_t* p) {
        return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
    }
    static uint32_t u32(const uint8_t* p) {
        return uint32_t(p[0]) | (uint32_t(p[1]) << 8)
            | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    }
    static float f32(const uint8_t* p) {
        static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
        const auto bits = u32(p);
        float value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }
    static bool decode(const uint8_t* p, size_t size, int gimbal_id, int shoot_id,
                       TongjiTelemetry& out) {
        // GimbalReceive: 3 float + 2 int32; ShootReceive: 1 float; two int16 IDs.
        if (p == nullptr || size != 28 || gimbal_id <= 0 || shoot_id <= 0
            || gimbal_id > 32767 || shoot_id > 32767 || gimbal_id == shoot_id)
            return false;
        TongjiTelemetry value{};
        bool have_gimbal = false, have_shoot = false;
        size_t offset = 0;
        while (offset < size) {
            if (size - offset < 2) return false;
            const auto id = u16(p + offset);
            offset += 2;
            if (id == gimbal_id && !have_gimbal && size - offset >= 20) {
                value.yaw = f32(p + offset);
                value.pitch = f32(p + offset + 4);
                value.roll = f32(p + offset + 8);
                const auto mode = u32(p + offset + 12);
                const auto robot_id = u32(p + offset + 16);
                if (mode > 255 || robot_id > 255) return false;
                value.mode = static_cast<int32_t>(mode);
                value.robot_id = static_cast<int32_t>(robot_id);
                have_gimbal = true;
                offset += 20;
            } else if (id == shoot_id && !have_shoot && size - offset >= 4) {
                value.bullet_speed = f32(p + offset);
                have_shoot = true;
                offset += 4;
            } else {
                return false;
            }
        }
        if (!have_gimbal || !have_shoot || !std::isfinite(value.yaw)
            || !std::isfinite(value.pitch) || !std::isfinite(value.roll)
            || !std::isfinite(value.bullet_speed) || value.bullet_speed < 0)
            return false;
        out = value;
        return true;
    }
};

// No sync marker and no CRC exist in the source protocol. Never scan arbitrary
// bytes for a plausible length: a framing fault requires a new connection.
class TelemetryStream {
public:
    using Clock = std::chrono::steady_clock;
    explicit TelemetryStream(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
        : timeout_(timeout) {
        if (timeout.count() <= 0) throw std::invalid_argument("frame timeout must be positive");
    }
    void reset() { used_ = 0; started_.reset(); failed_ = false; }
    bool expired(Clock::time_point now) const {
        return started_ && now - *started_ >= timeout_;
    }
    bool failed() const { return failed_; }
    std::size_t buffered() const { return used_; }
    template<class Callback>
    bool feed(const uint8_t* data, std::size_t size, Clock::time_point now, Callback on_frame) {
        if (failed_ || expired(now) || (data == nullptr && size != 0)) {
            failed_ = true;
            return false;
        }
        for (std::size_t i = 0; i < size; ++i) {
            if (used_ == 0) started_ = now;
            bytes_[used_++] = data[i];
            if (used_ == 2 && TongjiTelemetry::u16(bytes_.data()) != 28) {
                failed_ = true;
                return false;
            }
            if (used_ == bytes_.size()) {
                on_frame(bytes_.data() + 2, std::size_t(28));
                used_ = 0;
                started_.reset();
            }
        }
        return true;
    }
private:
    std::array<uint8_t, 30> bytes_{};
    std::size_t used_ = 0;
    std::optional<Clock::time_point> started_;
    std::chrono::milliseconds timeout_;
    bool failed_ = false;
};
} // namespace awakening
