#include "utils/drivers/command_protocol.hpp"
#include "utils/drivers/serial_driver.hpp"
#include "utils/drivers/vision_command_guard.hpp"
#include <cassert>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <iostream>
using namespace awakening;
using namespace std::chrono_literals;
using Bytes = std::vector<uint8_t>;
// Test-only encoder, never included in the application.
void u16(Bytes& b, unsigned v) { b.push_back(v & 255); b.push_back((v >> 8) & 255); }
void u32(Bytes& b, uint32_t v) { for (int i=0;i<4;++i) b.push_back((v >> (8*i)) & 255); }
void f32(Bytes& b, float v) { uint32_t bits; std::memcpy(&bits,&v,4); u32(b,bits); }
Bytes payload(bool reverse=false) {
    Bytes g,s; u16(g,7); f32(g,12.5f); f32(g,-2.5f); f32(g,0.5f); u32(g,1); u32(g,3);
    u16(s,9); f32(s,20.0f);
    if (reverse) { s.insert(s.end(),g.begin(),g.end()); return s; }
    g.insert(g.end(),s.begin(),s.end()); return g;
}
Bytes frame() { Bytes b; u16(b,28); auto p=payload(); b.insert(b.end(),p.begin(),p.end()); return b; }
template<class F> void wait_for(F f) {
    auto until=std::chrono::steady_clock::now()+3s;
    while (!f() && std::chrono::steady_clock::now()<until) std::this_thread::sleep_for(5ms);
    assert(f());
}
void protocol_tests() {
    TongjiTelemetry out{};
    for (bool rev : {false,true}) {
        auto p=payload(rev); assert(TongjiTelemetry::decode(p.data(),p.size(),7,9,out));
        assert(out.yaw==12.5f && out.pitch==-2.5f && out.bullet_speed==20 && out.robot_id==3);
    }
    // Firmware a6e1638 registers telemetry IDs 1/2 and stores uint8 robot_id in color.
    for (unsigned robot_id : {0u, 7u, 107u, 255u}) {
        auto actual=payload(); actual[0]=1; actual[22]=2; actual[18]=robot_id;
        actual[14]=255; // uint8 mode is logged raw, not interpreted as a control command.
        assert(TongjiTelemetry::decode(actual.data(),actual.size(),1,2,out));
        assert(out.robot_id==static_cast<int32_t>(robot_id) && out.mode==255);
    }
    auto p=payload();
    for (size_t n=0;n<28;++n) assert(!TongjiTelemetry::decode(p.data(),n,7,9,out));
    assert(!TongjiTelemetry::decode(nullptr,28,7,9,out));
    for (auto ids : {std::pair{-1,-1},std::pair{7,7},std::pair{0,9},std::pair{32768,9}})
        assert(!TongjiTelemetry::decode(p.data(),p.size(),ids.first,ids.second,out));
    for (size_t off : {size_t(0),size_t(22),size_t(15),size_t(19)}) {
        auto bad=p; bad[off]=255; assert(!TongjiTelemetry::decode(bad.data(),28,7,9,out));
    }
    auto bad=p; bad[22]=7; assert(!TongjiTelemetry::decode(bad.data(),28,7,9,out));
    for (size_t off : {size_t(2),size_t(6),size_t(10),size_t(24)}) {
        for (uint8_t mantissa : {uint8_t(128),uint8_t(192)}) {
            bad=p; bad[off]=0; bad[off+1]=0; bad[off+2]=mantissa; bad[off+3]=127;
            out.yaw=99; assert(!TongjiTelemetry::decode(bad.data(),28,7,9,out)); assert(out.yaw==99);
        }
    }
    bad=p; bad[27] |= 128; assert(!TongjiTelemetry::decode(bad.data(),28,7,9,out));
    auto wire=frame(); auto t=TelemetryStream::Clock::now();
    for (size_t split=0;split<=wire.size();++split) {
        TelemetryStream stream; int count=0;
        auto cb=[&](const uint8_t* d,size_t n){assert(TongjiTelemetry::decode(d,n,7,9,out)); ++count;};
        assert(stream.feed(wire.data(),split,t,cb));
        assert(stream.feed(wire.data()+split,wire.size()-split,t,cb));
        assert(count==1 && stream.buffered()==0);
    }
    TelemetryStream stream(50ms); int count=0;
    auto cb=[&](const uint8_t*,size_t){++count;};
    Bytes bulk; for(int i=0;i<10000;++i) bulk.insert(bulk.end(),wire.begin(),wire.end());
    assert(stream.feed(bulk.data(),bulk.size(),t,cb)); assert(count==10000 && stream.buffered()==0);
    for(unsigned length : {0u,1u,27u,29u,1025u,65535u}) {
        stream.reset(); Bytes wrong;u16(wrong,length); assert(!stream.feed(wrong.data(),2,t,cb));
        assert(!stream.feed(wire.data(),wire.size(),t,cb));
    }
    stream.reset(); assert(stream.feed(wire.data(),1,t,cb)); assert(!stream.expired(t+49ms));
    assert(stream.expired(t+50ms)); assert(!stream.feed(wire.data()+1,29,t+50ms,cb));
    stream.reset(); assert(stream.feed(wire.data(),wire.size(),t+60ms,cb));
    // No CRC: a bit change that leaves valid fields cannot be detected.
    bad=p;bad[2]^=1;assert(TongjiTelemetry::decode(bad.data(),28,7,9,out));
    std::cout<<"PASS: payload/fields, all split boundaries, 10000 joined frames, timeout/reset, no-CRC limitation\n";
}
// 上位机 -> 下位机 线格式：必须与 control-2026 的 InitParam() 注册一致（18 字节）
void command_tests() {
    const auto f = VisionCommand::encode(1.0f, -2.0f, true);
    assert(f.size() == 18);
    assert(f[0] == 0x10 && f[1] == 0x00);                          // total_len = 16
    assert(f[2] == 0x01 && f[3] == 0x00);                          // id 1
    assert(f[4] == 0x00 && f[5] == 0x00 && f[6] == 0x80 && f[7] == 0x3F);   // 1.0f
    assert(f[8] == 0x00 && f[9] == 0x00 && f[10] == 0x00 && f[11] == 0xC0); // -2.0f
    assert(f[12] == 0x02 && f[13] == 0x00);                        // id 2
    assert(f[14] == 0x01 && f[15] == 0x00 && f[16] == 0x00 && f[17] == 0x00); // fire_flag = 1
    const auto g = VisionCommand::encode(0.0f, 0.0f, false);
    assert(g.size() == 18);
    assert(g[14] == 0x00 && g[17] == 0x00);                        // fire_flag = 0
    assert(VisionCommand::encode_hold(1.0f, -2.0f) == VisionCommand::encode(1.0f, -2.0f, false));
    const auto zero_hold = VisionCommand::encode_hold(0.0f, 0.0f);
    assert(zero_hold != g && zero_hold[14] == 0x00 && zero_hold[17] == 0x00);
    assert(VisionCommand::encode_idle() == g);                     // 无目标时的空帧
    std::cout << "PASS: command frame layout (18B: len/id1/yaw/pitch/id2/fire)\n";
}
void command_guard_tests() {
    VisionCommandGuard::Params params;
    params.acquire_ms = 50;
    params.fire_settle_ms = 20;
    params.max_yaw_jump_deg = 4.0;
    params.max_pitch_jump_deg = 4.0;
    params.max_yaw_rate_deg_s = 1000.0;
    params.max_pitch_rate_deg_s = 1000.0;
    params.filter_time_constant_s = 0.0;
    params.fire_alignment_tolerance_deg = 0.1;
    VisionCommandGuard guard(params);
    const auto t = VisionCommandGuard::Clock::now();
    guard.seed(0.0, 0.0, t);
    VisionCommandGuard::Input input {
        .target = true,
        .target_key = 2,
        .yaw_deg = 10.0,
        .pitch_deg = -2.0,
        .distance_m = 5.0,
        .fire = true,
    };
    auto output = guard.update(input, t);
    assert(!output.tracking && !output.fire);
    output = guard.update(input, t + 49ms);
    assert(!output.tracking && output.yaw_deg == 0.0);
    output = guard.update(input, t + 50ms);
    assert(output.tracking && !output.fire && output.yaw_deg == 1.0);
    output = guard.update(input, t + 60ms);
    assert(output.tracking && !output.fire && output.yaw_deg == 10.0);
    output = guard.update(input, t + 71ms);
    assert(output.tracking && output.fire);

    input.yaw_deg = 20.0;
    output = guard.update(input, t + 72ms);
    assert(!output.tracking && !output.fire && output.state == VisionCommandGuard::State::AngleJump);
    input.distance_m = 0.12;
    output = guard.update(input, t + 73ms);
    assert(!output.tracking && output.state == VisionCommandGuard::State::InvalidMeasurement);
    input.distance_m = 5.0;
    input.yaw_deg = 10.0;
    output = guard.update(input, t + 74ms);
    assert(!output.tracking);
    input.target_key = 6;
    output = guard.update(input, t + 75ms);
    assert(!output.tracking && output.state == VisionCommandGuard::State::TargetChanged);
    input.target = false;
    output = guard.update(input, t + 76ms);
    assert(output.has_output && !output.tracking && !output.fire && output.yaw_deg == 10.0);
    std::cout << "PASS: command guard acquisition, range/jump/key rejection, slew and fire interlock\n";
}
struct Pty {
    int master; std::string slave;
    Pty() { master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK); assert(master>=0);
        assert(grantpt(master)==0 && unlockpt(master)==0);slave=ptsname(master); }
    ~Pty(){if(master>=0)close(master);}
};
Bytes read_for(int fd, std::chrono::milliseconds duration) {
    Bytes bytes;
    auto deadline=std::chrono::steady_clock::now()+duration;
    while(std::chrono::steady_clock::now()<deadline) {
        pollfd pfd{fd,POLLIN,0};
        if(poll(&pfd,1,5)<=0) continue;
        uint8_t chunk[512];
        const auto n=::read(fd,chunk,sizeof(chunk));
        if(n>0) bytes.insert(bytes.end(),chunk,chunk+n);
    }
    return bytes;
}
YAML::Node config(const std::string& device) {
    auto c=YAML::Load("baud_rate: 115200\nchar_size: 8\nread_buf_size: 64\ntelemetry_gimbal_id: 7\ntelemetry_shoot_id: 9\nframe_timeout_ms: 80\nidle_timeout_ms: 1000\nreconnect_ms: 30\n");
    c["device_name"]=device;return c;
}
void driver_tests() {
    Pty p; Scheduler scheduler; auto c=config(p.slave); SerialDriver d(c,scheduler);
    struct Tag{};d.start<Tag>("test");d.start<Tag>("duplicate");wait_for([&]{return d.statistics().connections==1;});
    // 发送：整帧必须被写到串口（可从 PTY 主端逐字节读回），不再返回 false
    const auto cmd=VisionCommand::encode(1.0f,-2.0f,true);
    bool wrote=false;
    for(int i=0;i<40&&!wrote;++i){wrote=d.write(cmd);if(!wrote)std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    assert(wrote);
    pollfd fd{p.master,POLLIN,0};assert(poll(&fd,1,200)>0);
    uint8_t tx[64]{};const ssize_t tx_len=::read(p.master,tx,sizeof(tx));
    assert(tx_len==(ssize_t)cmd.size());assert(std::memcmp(tx,cmd.data(),cmd.size())==0);
    // SrmLink-style TX: overwrite pending values, send at a fixed rate, then
    // fall back to the same angles with fire disabled after the heartbeat timeout.
    Pty latest;Scheduler latest_scheduler;auto latest_config=config(latest.slave);
    latest_config["command_tx_hz"]=100;latest_config["command_timeout_ms"]=60;
    SerialDriver latest_driver(latest_config,latest_scheduler);latest_driver.start<Tag>("latest");
    wait_for([&]{return latest_driver.statistics().connections==1;});
    const auto old_cmd=VisionCommand::encode(3.0f,4.0f,true);
    const auto newest_cmd=VisionCommand::encode(5.0f,6.0f,true);
    const auto safe_cmd=VisionCommand::encode_hold(5.0f,6.0f);
    assert(latest_driver.set_latest_command(old_cmd,VisionCommand::encode_hold(3.0f,4.0f)));
    assert(latest_driver.set_latest_command(newest_cmd,safe_cmd));
    auto scheduled=read_for(latest.master,45ms);
    assert(scheduled.size()>=newest_cmd.size()*2 && scheduled.size()%newest_cmd.size()==0);
    assert(std::memcmp(scheduled.data()+scheduled.size()-newest_cmd.size(),newest_cmd.data(),newest_cmd.size())==0);
    auto stale=read_for(latest.master,80ms);
    assert(stale.size()>=safe_cmd.size() && stale.size()%safe_cmd.size()==0);
    assert(std::memcmp(stale.data()+stale.size()-safe_cmd.size(),safe_cmd.data(),safe_cmd.size())==0);
    latest_driver.stop();
    // A reconnect must never replay the last armed frame, even when it occurs
    // before command_timeout_ms. Its first transmitted state keeps angles but clears fire.
    Pty reconnect;Scheduler reconnect_scheduler;auto reconnect_config=config(reconnect.slave);
    reconnect_config["command_tx_hz"]=10;reconnect_config["command_timeout_ms"]=500;
    reconnect_config["frame_timeout_ms"]=20;
    SerialDriver reconnect_driver(reconnect_config,reconnect_scheduler);
    reconnect_driver.start<Tag>("reconnect_safe");
    wait_for([&]{return reconnect_driver.statistics().connections==1;});
    assert(reconnect_driver.set_latest_command(newest_cmd,safe_cmd));
    auto armed=read_for(reconnect.master,130ms);
    assert(armed.size()>=newest_cmd.size() && armed.size()%newest_cmd.size()==0);
    assert(std::memcmp(armed.data()+armed.size()-newest_cmd.size(),newest_cmd.data(),newest_cmd.size())==0);
    const uint8_t partial=30;assert(::write(reconnect.master,&partial,1)==1);
    wait_for([&]{return reconnect_driver.statistics().connections>=2;});
    auto after_reconnect=read_for(reconnect.master,150ms);
    assert(after_reconnect.size()>=safe_cmd.size() && after_reconnect.size()%safe_cmd.size()==0);
    assert(std::memcmp(after_reconnect.data(),safe_cmd.data(),safe_cmd.size())==0);
    reconnect_driver.stop();
    auto wire=frame();assert(::write(p.master,wire.data(),wire.size())==30);
    wait_for([&]{return d.statistics().frames==1;});assert(d.statistics().invalid==0);
    const auto angles=d.latest_gimbal_angles();assert(angles && angles->first==12.5f && angles->second==-2.5f);
    // 只投递完整有效帧；业务层不应接收非法帧或跨连接半包。
    wait_for([&]{return scheduler.pushed_bytes==30;});
    assert(scheduler.pushed==1);
    auto bad=wire;bad[17]=1;assert(::write(p.master,bad.data(),bad.size())==30);
    wait_for([&]{return d.statistics().invalid==1;});
    assert(::write(p.master,wire.data(),3)==3);
    wait_for([&]{return d.statistics().timeouts>=1 && d.statistics().connections>=2;});
    assert(scheduler.pushed==1 && scheduler.pushed_bytes==30);
    assert(::write(p.master,wire.data(),wire.size())==30);wait_for([&]{return d.statistics().frames==3;});
    wait_for([&]{return scheduler.pushed_bytes==60;});
    assert(scheduler.pushed==2);
    const uint8_t corrupt[2]={255,255};assert(::write(p.master,corrupt,2)==2);
    wait_for([&]{return d.statistics().framing_errors>=1 && d.statistics().connections>=3;});
    assert(scheduler.pushed==2 && scheduler.pushed_bytes==60);
    assert(::write(p.master,wire.data(),wire.size())==30);
    wait_for([&]{return scheduler.pushed_bytes==90;});
    assert(scheduler.pushed==3);
    auto start=std::chrono::steady_clock::now();d.stop();assert(std::chrono::steady_clock::now()-start<500ms);
    d.stop();d.start<Tag>("restart");wait_for([&]{return d.statistics().connections>=4;});
    std::thread a([&]{d.stop();});std::thread b([&]{d.stop();});a.join();b.join();
    auto invalid=YAML::Clone(c);invalid["telemetry_shoot_id"]=7;bool threw=false;
    try {SerialDriver nope(invalid,scheduler);}catch(const std::invalid_argument&){threw=true;}assert(threw);
    auto idle=config(p.slave);idle["idle_timeout_ms"]=60;SerialDriver q(idle,scheduler);q.start<Tag>("idle");
    wait_for([&]{return q.statistics().timeouts>=1 && q.statistics().connections>=2;});q.stop();
    Pty unplug;SerialDriver r(config(unplug.slave),scheduler);r.start<Tag>("unplug");
    wait_for([&]{return r.statistics().connections==1;});close(unplug.master);unplug.master=-1;
    std::this_thread::sleep_for(100ms);start=std::chrono::steady_clock::now();r.stop();
    assert(std::chrono::steady_clock::now()-start<500ms);
    // 未配置同济 ID 的旧协议串口仍须透传任意字节。
    Pty raw; Scheduler raw_scheduler; auto raw_config=config(raw.slave);
    raw_config["telemetry_gimbal_id"]=-1;raw_config["telemetry_shoot_id"]=-1;
    SerialDriver raw_driver(raw_config,raw_scheduler);raw_driver.start<Tag>("raw");
    wait_for([&]{return raw_driver.statistics().connections==1;});
    const uint8_t arbitrary[]={0xff,0x00,0x11};
    assert(::write(raw.master,arbitrary,sizeof(arbitrary))==3);
    wait_for([&]{return raw_scheduler.pushed_bytes==3;});raw_driver.stop();
    std::cout<<"PASS: PTY command TX + telemetry RX/publish, reject, timeout/framing reconnect, start/restart/concurrent stop, invalid config, idle timeout, unplug\n";
}
int main(){protocol_tests();command_tests();command_guard_tests();driver_tests();}
