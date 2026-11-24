#pragma once

#include "can_tester.hpp"
#include "serial_tester.hpp"

#include <string>

class RkTesterEngine;

// 配置持久化（记住最后一次选择的测试项、网口过滤与总线方向）
struct SavedConfig {
    bool loaded{false};
    bool cpu{false};
    bool gpu{false};
    bool npu{false};
    bool rga{false};
    bool mpp{false};
    bool ddr{false};
    bool emc{false};
    bool lan_highload{false};
    bool rs232{false};
    bool rs485{false};
    bool usb{false};
    bool can{false};
    bool can_fd{false};
    bool ports[4]{true, true, true, true};
    SerialDirection rs232_dir{SerialDirection::A_TO_B};
    SerialDirection rs485_dir{SerialDirection::A_TO_B};
    CanDirection can_dir{CanDirection::CAN0_TO_CAN1};
    bool governor_perf{false};
};

std::string get_config_path();
SavedConfig load_config(const std::string& path);
bool save_config_snapshot(const std::string& path, const SavedConfig& cfg);
void save_engine_config(const RkTesterEngine& engine, const std::string& path);
