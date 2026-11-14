#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace awakening {
// control-2026 (stm32-f4 / sentry_omni_gimbal) 上位机->下位机线格式：
//   int16 total_len(=各记录 2+载荷 之和) | (int16 id, payload)*
//   id 1 -> {float yaw; float pitch;}   id 2 -> {int32 fire_flag;}
// 全小端，无 CRC，无握手。必须「一次 write 恰好一帧 18 字节」。
// 这只是发送接口约束，不能保证 USB 回调边界；固件仍需按实际接收长度处理拆包/粘包。
class VisionCommand {
public:
    static constexpr int16_t kGimbalId = 1;
    static constexpr int16_t kShootId = 2;
    static constexpr std::size_t kFrameSize = 18;
    static constexpr float kZeroHoldNudgeDeg = 0.0001f;
    static std::vector<uint8_t> encode(float yaw_deg, float pitch_deg, bool fire) {
        std::vector<uint8_t> out;
        out.reserve(kFrameSize);
        put16(out, 16);                       // (2+8) + (2+4)
        put16(out, kGimbalId);
        put_f32(out, yaw_deg);
        put_f32(out, pitch_deg);
        put16(out, kShootId);
        put_i32(out, fire ? 1 : 0);
        return out;
    }
    static std::vector<uint8_t> encode_hold(float yaw_deg, float pitch_deg) {
        // Firmware uses an all-zero payload as the only idle marker. Preserve
        // hold semantics at the exact origin with a mechanically negligible nudge.
        if (yaw_deg == 0.0f && pitch_deg == 0.0f) yaw_deg = kZeroHoldNudgeDeg;
        return encode(yaw_deg, pitch_deg, false);
    }
    // 仅在尚未产生任何有效角度时使用。已锁定过目标后应使用 encode_hold，
    // 避免短暂丢帧在视觉/遥控模式之间反复切换。
    static std::vector<uint8_t> encode_idle() {
        return encode(0.0f, 0.0f, false);
    }
private:
    static void put16(std::vector<uint8_t>& b, int16_t v) {
        b.push_back(static_cast<uint8_t>(v & 0xFF));
        b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    }
    static void put_u32(std::vector<uint8_t>& b, uint32_t v) {
        for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
    }
    static void put_f32(std::vector<uint8_t>& b, float v) {
        uint32_t u; std::memcpy(&u, &v, sizeof(u)); put_u32(b, u);
    }
    static void put_i32(std::vector<uint8_t>& b, int32_t v) {
        put_u32(b, static_cast<uint32_t>(v));
    }
};
} // namespace awakening
