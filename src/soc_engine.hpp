#pragma once

#include "can_tester.hpp"
#include "net_emc.hpp"
#include "serial_tester.hpp"
#include "usb_tester.hpp"

#include <atomic>
#include <cstddef>
#include <map>
#include <string>
#include <sys/types.h>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

struct CpuTime {
    unsigned long long total{0};
    unsigned long long idle{0};
};

class RkTesterEngine {
public:
    int num_cores{8};
    std::vector<CpuTime> prev_cores;
    CpuTime prev_total;
    std::vector<double> smooth_cores;
    double smooth_total{0.0};
    bool cpu_initialized{false};
    std::map<std::string, std::string> thermal_zones;

    // 压测状态
    std::atomic<bool> cpu_stress{false};
    std::atomic<bool> gpu_stress{false};
    std::atomic<bool> npu_stress{false};
    int target_cpu_cores{8};
    bool governor_perf{false};

    std::vector<std::thread> cpu_threads;
    pid_t gpu_pid{0};
    std::vector<pid_t> npu_pids;

    // 4 路千兆网卡 EMC 丢包监测引擎
    NetEmcTester net_emc;

    // 串口测试（支持单向正向/反向与半双工 Ping-Pong）
    SerialChannel rs232{"RS232", "S0", "S1", "/dev/ttyS0", "/dev/ttyS1", "Z"};
    SerialChannel rs485{"RS485", "S2", "S3", "/dev/ttyS2", "/dev/ttyS3", "X"};

    // 4 路 USB 存储读写压测
    UsbStorageTester usb_tester;

    // CAN0 <-> CAN1 互发测试
    CanBusTester can_tester;

    RkTesterEngine();
    ~RkTesterEngine();

    void toggle_rs232();
    void toggle_rs485();
    void toggle_usb_stress();
    void toggle_can();

    EmcSnapshot get_emc_snapshot() const;
    void set_port_enabled(size_t idx, bool en);
    bool get_port_enabled(size_t idx) const;
    void log_emc_event(const std::string& level, const std::string& msg, int color);
    void setup_default_network();

    void reset_all_stats();

    void toggle_cpu_stress();
    std::string toggle_gpu_stress();
    std::string toggle_npu_stress();
    std::string toggle_all_stress();
    std::string check_stress_health();
    void stop_all_stress();
    void toggle_governor();

    std::pair<double, std::vector<double>> get_cpu_loads();
    std::vector<int> get_cpu_freqs() const;
    std::pair<double, int> get_gpu_stats() const;
    std::pair<std::vector<double>, int> get_npu_stats() const;
    std::map<std::string, double> get_thermals() const;
    std::tuple<double, double, double> get_mem_stats() const;
    std::string get_uptime() const;

private:
    void init_governor_state();
    void init_thermals();
};
