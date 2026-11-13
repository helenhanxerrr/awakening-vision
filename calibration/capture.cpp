// Adapted from TongjiSuperPower/sp_vision_25/calibration/capture.cpp.
// Copyright (c) 2025 TongjiSuperPower. See LICENSE-TongjiSuperPower.
#include "utils/drivers/camera_factory.hpp"
#include "utils/drivers/serial_driver.hpp"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numbers>
#include <optional>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace awakening;
namespace fs = std::filesystem;
namespace {
volatile std::sig_atomic_t running = 1;
void stop_signal(int) { running = 0; }
struct PoseTag {};
using PoseIO = IOPair<PoseTag, std::vector<uint8_t>>;
struct Pose { TimePoint time; Quaternion q; uint64_t connection; };

class PoseBuffer {
    std::mutex mutex_;
    std::deque<Pose> poses_;
public:
    void push(Pose pose) {
        std::lock_guard lock(mutex_);
        if (!poses_.empty() && poses_.back().connection != pose.connection) poses_.clear();
        poses_.push_back(std::move(pose));
        while (poses_.size() > 2048) poses_.pop_front();
    }
    std::optional<Pose> stationary_at(TimePoint time, uint64_t connection,
                                    double max_age_ms, double window_ms, double angle_deg) {
        std::lock_guard lock(mutex_);
        std::vector<Pose> recent;
        for (const auto& p : poses_) {
            const double dt = std::chrono::duration<double, std::milli>(time - p.time).count();
            if (p.connection == connection && dt >= -max_age_ms && dt <= window_ms)
                recent.push_back(p);
        }
        if (recent.size() < 3) return std::nullopt;
        const auto nearest = std::min_element(recent.begin(), recent.end(), [&](auto& a, auto& b) {
            return std::abs((time-a.time).count()) < std::abs((time-b.time).count());
        });
        if (std::abs(std::chrono::duration<double, std::milli>(time-nearest->time).count()) > max_age_ms)
            return std::nullopt;
        const auto span = std::chrono::duration<double, std::milli>(recent.back().time-recent.front().time).count();
        if (span < window_ms * .7) return std::nullopt;
        for (const auto& p : recent)
            if (p.q.angularDistance(nearest->q) * 180. / std::numbers::pi > angle_deg)
                return std::nullopt;
        return *nearest;
    }
};

YAML::Node select_camera(const YAML::Node& root, const std::string& key) {
    YAML::Node node = YAML::Clone(root);
    std::istringstream path(key);
    std::string part;
    while (std::getline(path, part, '.')) {
        if (!node.IsMap() || !node[part]) throw std::runtime_error("Missing camera key: " + key);
        node.reset(node[part]);
    }
    return node;
}

// Only create files inside a newly reserved sample directory, then rename into
// the session folder. Interrupted pairs are never seen by the offline loader.
void save_pair(const fs::path& folder, size_t index, const cv::Mat& image, const std::optional<Pose>& pose) {
    const auto stem = std::to_string(index);
    const auto pending = folder / (".pending-" + stem);
    if (!fs::create_directory(pending)) throw std::runtime_error("Sample already pending");
    const auto image_path = folder / (stem + ".png");
    const auto pose_path = folder / (stem + ".txt");
    if (fs::exists(image_path) || fs::exists(pose_path)) throw std::runtime_error("Sample already exists");
    if (!cv::imwrite((pending / "image.png").string(), image)) throw std::runtime_error("Image write failed");
    if (pose) {
        std::ofstream file(pending / "pose.txt");
        file << std::setprecision(17) << pose->q.w() << ' ' << pose->q.x() << ' '
             << pose->q.y() << ' ' << pose->q.z() << '\n';
        file.close();
        if (!file) throw std::runtime_error("Quaternion write failed");
        fs::rename(pending / "pose.txt", pose_path);
    }
    // Publish image last. An orphan txt is harmless if interrupted here.
    fs::rename(pending / "image.png", image_path);
    fs::remove(pending);
}
} // namespace

int main(int argc, char** argv) {
    const std::string keys =
        "{help h ? | | Show help}"
        "{@robot-config | config/sentry.yaml | Awakening robot YAML}"
        "{settings | config/calibration.yaml | Board and pose settings}"
        "{output o | output/calibration/session | NEW session directory}"
        "{camera-key | camera | Camera mapping path}"
        "{intrinsics-only | false | Save images without opening serial}"
        "{save-unrecognized | false | Allow unrecognized raw images in intrinsics-only mode}"
        "{headless | false | No GUI; requires interval and count}"
        "{interval | 0 | Automatic capture interval in seconds; zero means press s}"
        "{count | 0 | Stop after this many saved images; zero means unlimited}";
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) { cli.printMessage(); return 0; }
    try {
        const auto config_path = cli.get<std::string>(0);
        const auto settings_path = cli.get<std::string>("settings");
        const auto output = fs::path(cli.get<std::string>("output"));
        const auto camera_key = cli.get<std::string>("camera-key");
        const bool only_images = cli.get<bool>("intrinsics-only");
        const bool save_unrecognized = cli.get<bool>("save-unrecognized");
        const bool headless = cli.get<bool>("headless");
        const double interval = cli.get<double>("interval");
        const int count = cli.get<int>("count");
        if (!cli.check()) { cli.printErrors(); return 1; }
        if (!std::isfinite(interval) || interval < 0 || count < 0 || (headless && (interval <= 0 || count <= 0)))
            throw std::runtime_error("Invalid interval/count; headless requires positive interval and count");
        const auto config = YAML::LoadFile(config_path);
        const auto settings = YAML::LoadFile(settings_path);
        const auto pattern_type = settings["pattern_type"].as<std::string>("circles");
        if (pattern_type != "circles" && pattern_type != "chessboard")
            throw std::runtime_error("pattern_type must be circles or chessboard");
        if (save_unrecognized && !only_images)
            throw std::runtime_error("save-unrecognized requires intrinsics-only=true");
        const cv::Size pattern(settings["pattern_cols"].as<int>(), settings["pattern_rows"].as<int>());
        const double max_age = settings["pose_max_age_ms"].as<double>(100);
        const double window = settings["pose_stable_window_ms"].as<double>(300);
        const double angle = settings["pose_stable_angle_deg"].as<double>(.3);
        if (pattern.width < 2 || pattern.height < 2 || !std::isfinite(max_age) || max_age <= 0
            || !std::isfinite(window) || window < max_age || !std::isfinite(angle) || angle <= 0)
            throw std::runtime_error("Invalid board/pose settings");
        if (!only_images) {
            if (camera_key != "camera") throw std::runtime_error("Pose capture supports the main camera only");
            auto mount = settings["R_gimbal2imubody"].as<std::vector<double>>();
            const std::vector<double> identity{1,0,0,0,1,0,0,0,1};
            if (mount != identity) throw std::runtime_error("Live poses are already gimbal->world; use identity R_gimbal2imubody");
            if (!config["serial"]["enable"].as<bool>(false)
                || config["serial"]["telemetry_gimbal_id"].as<int>(-1) <= 0
                || config["serial"]["telemetry_shoot_id"].as<int>(-1) <= 0)
                throw std::runtime_error("Hand-eye capture needs enabled telemetry with valid IDs; use --intrinsics-only otherwise");
        }
        // Never reuse a session: image/pose provenance and sample numbering stay unambiguous.
        if (!output.parent_path().empty()) fs::create_directories(output.parent_path());
        if (!fs::create_directory(output)) throw std::runtime_error("Output session already exists");
        fs::copy_file(config_path, output / "robot-config.yaml");
        fs::copy_file(settings_path, output / "calibration-settings.yaml");
        PoseBuffer buffer;
        Scheduler scheduler(1);
        std::unique_ptr<SerialDriver> serial;
        // RAII must stop scheduled callbacks before buffer/serial are destroyed, even on exceptions.
        struct StopScheduler { Scheduler& s; ~StopScheduler() { s.stop(); } } stop_scheduler{scheduler};
        auto camera = create_camera(select_camera(config, camera_key), scheduler, "hik");
        if (!only_images) {
            serial = std::make_unique<SerialDriver>(config["serial"], scheduler);
            const int gid = serial->params_.telemetry_gimbal_id, sid = serial->params_.telemetry_shoot_id;
            const auto delay = std::chrono::microseconds(config["serial_send_to_image_microseconds"].as<int>(0));
            scheduler.register_task<PoseIO>("calibration_pose", [&, gid, sid, delay](std::vector<uint8_t>&& bytes) {
                TongjiTelemetry state{};
                if (bytes.size() != 30 || TongjiTelemetry::u16(bytes.data()) != 28
                    || !TongjiTelemetry::decode(bytes.data()+2, 28, gid, sid, state)) return;
                const double rad = std::numbers::pi / 180.;
                Quaternion q = AngleAxis(state.yaw*rad, Vec3::UnitZ())
                             * AngleAxis(state.pitch*rad, Vec3::UnitY())
                             * AngleAxis(state.roll*rad, Vec3::UnitX());
                buffer.push({Clock::now()-delay, q.normalized(), serial->statistics().connections});
            });
            serial->start<PoseTag>("calibration_serial");
            scheduler.build();
            scheduler.run();
        }
        camera->init();
        if (!camera->start_capture()) throw std::runtime_error("Camera capture failed to start");
        std::signal(SIGINT, stop_signal);
        std::signal(SIGTERM, stop_signal);
        std::cout << "s: save, q/Esc: quit. Stop gimbal motion before saving.\n";
        auto last_save = std::chrono::steady_clock::now();
        size_t saved = 0;
        int failures = 0;
        while (running && camera->is_running()) {
            auto frame = camera->read();
            if (frame.src_img.empty()) {
                if (++failures > 100) throw std::runtime_error("Camera produced 100 empty frames");
                continue;
            }
            failures = 0;
            cv::Mat image = frame.src_img;
            if (frame.format == PixelFormat::RGB) cv::cvtColor(frame.src_img, image, cv::COLOR_RGB2BGR);
            std::vector<cv::Point2f> centers;
            const bool found = pattern_type == "chessboard"
                ? cv::findChessboardCornersSB(image, pattern, centers, cv::CALIB_CB_NORMALIZE_IMAGE)
                : cv::findCirclesGrid(image, pattern, centers, cv::CALIB_CB_SYMMETRIC_GRID);
            auto pose = only_images ? std::optional<Pose>{} : buffer.stationary_at(
                frame.timestamp, serial->statistics().connections, max_age, window, angle);
            int key = -1;
            if (!headless) {
                auto drawing = image.clone();
                cv::drawChessboardCorners(drawing, pattern, centers, found);
                const auto status = only_images ? "images only" : (pose ? "pose ready" : "pose missing/moving");
                cv::putText(drawing, status, {20,40}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,0,255}, 2);
                cv::putText(drawing, pattern_type + (found ? ": detected" : ": not detected"),
                            {20,80}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,0,255}, 2);
                cv::resize(drawing, drawing, {}, .5, .5);
                cv::imshow("Calibration: s save, q quit", drawing);
                key = cv::waitKey(1) & 255;
            }
            if (key == 'q' || key == 27) break;
            const auto now = std::chrono::steady_clock::now();
            const bool automatic = interval > 0 && std::chrono::duration<double>(now-last_save).count() >= interval;
            if (key != 's' && !automatic) continue;
            if ((!found && !save_unrecognized) || (!only_images && !pose)) {
                if (key == 's') {
                    if (!found) std::cerr << "Not saved: board not detected; check pattern_type, cols/rows, lighting\n";
                    if (!only_images && !pose) std::cerr << "Not saved: need fresh stationary pose\n";
                }
                continue;
            }
            if (!found) std::cerr << "Saving unrecognized raw image; offline calibration may skip it\n";
            save_pair(output, saved+1, image, pose);
            ++saved;
            last_save = now;
            std::cout << "Saved " << saved << " in " << output << '\n';
            if (count > 0 && saved >= static_cast<size_t>(count)) break;
        }
        camera->stop();
        if (serial) serial->stop();
        scheduler.stop();
        if (!headless) cv::destroyAllWindows();
        if (saved == 0 || (count > 0 && saved < static_cast<size_t>(count))) {
            std::cerr << "Capture ended before requested sample count was collected\n";
            return 1;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Capture failed: " << error.what() << '\n';
        return 1;
    }
}
