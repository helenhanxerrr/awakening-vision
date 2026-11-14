#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace awakening {

class VisionCommandGuard {
public:
    using Clock = std::chrono::system_clock;

    struct Params {
        int acquire_ms = 120;
        int fire_settle_ms = 150;
        double min_distance_m = 0.5;
        double max_distance_m = 20.0;
        double max_yaw_jump_deg = 4.0;
        double max_pitch_jump_deg = 4.0;
        double max_yaw_rate_deg_s = 90.0;
        double max_pitch_rate_deg_s = 60.0;
        double filter_time_constant_s = 0.03;
        double fire_alignment_tolerance_deg = 0.5;
    };

    struct Input {
        bool target = false;
        int target_key = -1;
        double yaw_deg = 0.0;
        double pitch_deg = 0.0;
        double distance_m = 0.0;
        bool fire = false;
    };

    enum class State {
        NoTarget,
        InvalidMeasurement,
        Acquiring,
        TargetChanged,
        AngleJump,
        Tracking,
    };

    struct Output {
        bool has_output = false;
        bool tracking = false;
        bool fire = false;
        double yaw_deg = 0.0;
        double pitch_deg = 0.0;
        State state = State::NoTarget;
    };

    explicit VisionCommandGuard(Params params): params_(params) {
        if (params_.acquire_ms < 0 || params_.fire_settle_ms < 0
            || params_.min_distance_m <= 0.0
            || params_.max_distance_m <= params_.min_distance_m
            || params_.max_yaw_jump_deg <= 0.0 || params_.max_pitch_jump_deg <= 0.0
            || params_.max_yaw_rate_deg_s <= 0.0 || params_.max_pitch_rate_deg_s <= 0.0
            || params_.filter_time_constant_s < 0.0
            || params_.fire_alignment_tolerance_deg < 0.0)
        {
            throw std::invalid_argument("invalid vision command guard parameters");
        }
    }

    void seed(double yaw_deg, double pitch_deg, Clock::time_point now) noexcept {
        if (has_output_ || !std::isfinite(yaw_deg) || !std::isfinite(pitch_deg))
            return;
        output_yaw_deg_ = normalize_yaw(yaw_deg);
        output_pitch_deg_ = pitch_deg;
        last_output_update_ = now;
        has_output_ = true;
    }

    [[nodiscard]] bool has_output() const noexcept {
        return has_output_;
    }

    Output update(const Input& input, Clock::time_point now) noexcept {
        if (!input.target) {
            reset_acquisition();
            return hold(State::NoTarget, now);
        }
        if (!valid(input)) {
            reset_acquisition();
            return hold(State::InvalidMeasurement, now);
        }

        if (!have_candidate_) {
            start_candidate(input, now);
            return hold(State::Acquiring, now);
        }

        if (input.target_key != candidate_key_) {
            start_candidate(input, now);
            return hold(State::TargetChanged, now);
        }

        if (std::abs(shortest_yaw_delta(last_input_yaw_deg_, input.yaw_deg))
                > params_.max_yaw_jump_deg
            || std::abs(input.pitch_deg - last_input_pitch_deg_)
                > params_.max_pitch_jump_deg)
        {
            start_candidate(input, now);
            return hold(State::AngleJump, now);
        }

        last_input_yaw_deg_ = normalize_yaw(input.yaw_deg);
        last_input_pitch_deg_ = input.pitch_deg;
        if (!tracking_) {
            if (now - candidate_since_ < std::chrono::milliseconds(params_.acquire_ms))
                return hold(State::Acquiring, now);
            tracking_ = true;
            tracking_since_ = now;
        }

        if (!has_output_) {
            output_yaw_deg_ = normalize_yaw(input.yaw_deg);
            output_pitch_deg_ = input.pitch_deg;
            last_output_update_ = now;
            has_output_ = true;
        } else {
            const double elapsed_s = std::clamp(
                std::chrono::duration<double>(now - last_output_update_).count(),
                0.0,
                0.05
            );
            output_yaw_deg_ = normalize_yaw(
                output_yaw_deg_
                + filtered_step(
                    shortest_yaw_delta(output_yaw_deg_, input.yaw_deg),
                    params_.max_yaw_rate_deg_s,
                    elapsed_s
                )
            );
            output_pitch_deg_ += filtered_step(
                input.pitch_deg - output_pitch_deg_,
                params_.max_pitch_rate_deg_s,
                elapsed_s
            );
            last_output_update_ = now;
        }

        const bool aligned =
            std::abs(shortest_yaw_delta(output_yaw_deg_, input.yaw_deg))
                <= params_.fire_alignment_tolerance_deg
            && std::abs(output_pitch_deg_ - input.pitch_deg)
                <= params_.fire_alignment_tolerance_deg;
        const bool settled =
            now - tracking_since_ >= std::chrono::milliseconds(params_.fire_settle_ms);
        return {
            .has_output = true,
            .tracking = true,
            .fire = input.fire && aligned && settled,
            .yaw_deg = output_yaw_deg_,
            .pitch_deg = output_pitch_deg_,
            .state = State::Tracking,
        };
    }

    static const char* state_name(State state) noexcept {
        switch (state) {
        case State::NoTarget: return "no_target";
        case State::InvalidMeasurement: return "invalid_measurement";
        case State::Acquiring: return "acquiring";
        case State::TargetChanged: return "target_changed";
        case State::AngleJump: return "angle_jump";
        case State::Tracking: return "tracking";
        }
        return "unknown";
    }

private:
    static double normalize_yaw(double angle_deg) noexcept {
        return std::remainder(angle_deg, 360.0);
    }

    static double shortest_yaw_delta(double from_deg, double to_deg) noexcept {
        return std::remainder(to_deg - from_deg, 360.0);
    }

    bool valid(const Input& input) const noexcept {
        return std::isfinite(input.yaw_deg) && std::isfinite(input.pitch_deg)
            && std::isfinite(input.distance_m)
            && input.distance_m >= params_.min_distance_m
            && input.distance_m <= params_.max_distance_m;
    }

    void start_candidate(const Input& input, Clock::time_point now) noexcept {
        have_candidate_ = true;
        tracking_ = false;
        candidate_key_ = input.target_key;
        candidate_since_ = now;
        last_input_yaw_deg_ = normalize_yaw(input.yaw_deg);
        last_input_pitch_deg_ = input.pitch_deg;
    }

    void reset_acquisition() noexcept {
        have_candidate_ = false;
        tracking_ = false;
        candidate_key_ = -1;
    }

    double filtered_step(double error_deg, double rate_deg_s, double elapsed_s) const noexcept {
        if (elapsed_s <= 0.0)
            return 0.0;
        const double alpha = params_.filter_time_constant_s > 0.0
            ? 1.0 - std::exp(-elapsed_s / params_.filter_time_constant_s)
            : 1.0;
        return std::clamp(error_deg * alpha, -rate_deg_s * elapsed_s, rate_deg_s * elapsed_s);
    }

    Output hold(State state, Clock::time_point now) noexcept {
        if (has_output_)
            last_output_update_ = now;
        return {
            .has_output = has_output_,
            .tracking = false,
            .fire = false,
            .yaw_deg = output_yaw_deg_,
            .pitch_deg = output_pitch_deg_,
            .state = state,
        };
    }

    Params params_;
    bool have_candidate_ = false;
    bool tracking_ = false;
    bool has_output_ = false;
    int candidate_key_ = -1;
    double last_input_yaw_deg_ = 0.0;
    double last_input_pitch_deg_ = 0.0;
    double output_yaw_deg_ = 0.0;
    double output_pitch_deg_ = 0.0;
    Clock::time_point candidate_since_ {};
    Clock::time_point tracking_since_ {};
    Clock::time_point last_output_update_ {};
};

} // namespace awakening
