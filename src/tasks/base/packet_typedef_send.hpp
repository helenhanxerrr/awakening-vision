#pragma once
// Legacy protocol types retained for existing callers and indirect sentry_brain
// dependencies. These are NOT the current sentry wire format.
//
// 精确关系：sentry 的**主串口收发路径**已改用 utils/drivers/command_protocol.hpp（发送）
// 与 utils/drivers/telemetry_protocol.hpp（接收），不再用本文件的类型组帧；
// 但 sentry_brain 仍在本文件里取用 SentryRefereeSend 等类型
// （home_mode.hpp / dog_mode.hpp 的 send_current_pose()），因此**编译时不能删除本文件**。
// 注意：“被引用”不等于“会发出旧协议”——这两个模式类只构造对象，没有调用串口写入。

#include "tasks/base/web.hpp"
#include "utils/common/type_common.hpp"
#include "utils/utils.hpp"
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <utility>
#include <vector>
namespace awakening {

constexpr const char* TARGET_TOPIC = "vision_target";
constexpr const char* NAV_STATE_TOPIC = "rose_state";
constexpr const char* MODE_TOPIC = "sentry_mode";
constexpr const char* ROBO_STATE_TOPIC = "robo_state";
constexpr const char* GOAL_TOPIC = "rose_goal";

struct SendRobotCmdData {
    static constexpr uint8_t ID = 0x01;

    uint8_t cmd_ID = ID;
    uint32_t time_stamp;

    uint8_t appear;

    float pitch, yaw;
    float target_yaw, target_pitch;

    float enable_yaw_diff, enable_pitch_diff;
    float v_yaw, v_pitch;
    float a_yaw, a_pitch;

    uint8_t detect_color;
    auto to_tuple() const {
        return std::tie(
            cmd_ID,
            time_stamp,
            appear,
            pitch,
            yaw,
            target_yaw,
            target_pitch,
            enable_yaw_diff,
            enable_pitch_diff,
            v_yaw,
            v_pitch,
            a_yaw,
            a_pitch,
            detect_color
        );
    }

} __attribute__((packed));

struct SendNavCmdData {
    static constexpr uint8_t ID = 0x02;

    uint8_t cmd_ID = ID;
    float vx, vy, wz;
    uint8_t turtle_state;

} __attribute__((packed));

struct SentryRefereeSend {
    static constexpr uint8_t ID = 0x03;
    uint8_t cmd_ID = ID;
    uint8_t set_current_pose; //1 为进攻姿态，2 为防御姿态，3 为移动姿态
} __attribute__((packed));
struct SentryySendYUNTAISHOU {
    static constexpr uint8_t ID = 0x04;
    uint8_t cmd_ID = ID;
    uint8_t user_data[30];
} __attribute__((packed));
struct ShootControlSend {
    static constexpr uint8_t ID = 0x05;
    uint8_t cmd_ID = ID;
    uint32_t shoot_count = 0;
    auto to_tuple() const {
        return std::tie(cmd_ID, shoot_count);
    }
};
} // namespace awakening
