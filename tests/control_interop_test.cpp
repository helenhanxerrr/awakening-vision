#include "utils/drivers/command_protocol.hpp"
#include "utils/drivers/telemetry_protocol.hpp"
extern "C" {
#include "master_process.h"
#include "srm_protocol.h"
}
#include <array>
#include <cassert>
#include <cstring>
#include <iostream>
using namespace awakening;

int main() {
    static_assert(sizeof(short) == 2 && sizeof(int) == 4 && sizeof(float) == 4);
    static_assert(sizeof(Gimbal_Receive_s) == 8 && sizeof(Shoot_Receive_s) == 4);
    static_assert(sizeof(Gimbal_Send_s) == 20 && sizeof(Shoot_Send_s) == 4);
    Vision_Receive_s received{};
    Message rx{};
    rx.ptr_list[1] = &received.gimbal_receive;
    rx.size_list[1] = sizeof(received.gimbal_receive);
    rx.ptr_list[2] = &received.shoot_receive;
    rx.size_list[2] = sizeof(received.shoot_receive);
    const auto first = VisionCommand::encode(12.5f, -2.5f, true);
    const auto second = VisionCommand::encode(30.0f, 4.0f, false);
    std::array<uint8_t, 64> usb{};
    std::memcpy(usb.data(), first.data(), first.size());
    get_srm_protocol_info(usb.data(), &rx);
    assert(received.gimbal_receive.yaw == 12.5f);
    assert(received.gimbal_receive.pitch == -2.5f && received.shoot_receive.fire_flag == 1);

    Vision_Send_s sent{{12.5f, -2.5f, 0.5f, 1, 107}, {22.0f}};
    Message tx{};
    tx.ptr_list[1] = &sent.gimbal_send;
    tx.size_list[1] = sizeof(sent.gimbal_send);
    tx.ptr_list[2] = &sent.shoot_send;
    tx.size_list[2] = sizeof(sent.shoot_send);
    uint16_t length = 0;
    srm_protocol_pack_send_data(&tx, usb.data(), &length);
    assert(length == 30);
    for (size_t split = 0; split <= length; ++split) {
        TelemetryStream stream;
        int count = 0;
        const auto now = TelemetryStream::Clock::now();
        auto decode = [&](const uint8_t* p, size_t n) {
            TongjiTelemetry state{};
            assert(TongjiTelemetry::decode(p, n, 1, 2, state));
            assert(state.yaw == 12.5f && state.pitch == -2.5f && state.roll == 0.5f);
            assert(state.mode == 1 && state.robot_id == 107 && state.bullet_speed == 22.0f);
            ++count;
        };
        assert(stream.feed(usb.data(), split, now, decode));
        assert(stream.feed(usb.data() + split, length - split, now, decode));
        assert(count == 1);
    }
    std::cout << "PASS: vision command -> actual firmware decoder; actual firmware telemetry -> vision (all splits)\n";

    // Reproduce upstream limitations, not successful transport guarantees.
    usb.fill(0);
    std::memcpy(usb.data(), first.data(), first.size());
    std::memcpy(usb.data() + first.size(), second.data(), second.size());
    get_srm_protocol_info(usb.data(), &rx);
    assert(received.gimbal_receive.yaw == 12.5f); // second frame discarded
    usb.fill(0);
    std::memcpy(usb.data(), first.data(), 8); // short callback, stale/zero tail
    get_srm_protocol_info(usb.data(), &rx);
    assert(received.gimbal_receive.yaw == 12.5f);
    assert(received.gimbal_receive.pitch != -2.5f); // ignores actual received size
    std::cout << "CONFIRMED LIMITATION: firmware drops joined commands and consumes short packets as complete\n";
}
