#pragma once

#include "can_tester.hpp"
#include "ddr_tester.hpp"
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

struct MppStats {
    double enc_fps{0.0};
    double dec_fps{0.0};
    int enc_freq_mhz{0};
    int dec_freq_mhz{0};
    uint64_t enc_total{0};
    uint64_t dec_total{0};
};

struct ThermalThrottleState {
    bool cpu_little{false};
    bool cpu_big{false};
    bool gpu{false};
    bool npu{false};
    int gpu_cur{0};
    int gpu_max{0};
    bool any() const { return cpu_little || cpu_big || gpu || npu; }
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
    std::map<std::string, std::string> cooling_devs;

    // 压测状态
    std::atomic<bool> cpu_stress{false};
    std::atomic<bool> gpu_stress{false};
    std::atomic<bool> npu_stress{false};
    std::atomic<bool> rga_stress{false};
    std::atomic<bool> mpp_stress{false};
    int target_cpu_cores{8};
    bool keep_isolcpus{false};
    std::vector<int> cpu_burn_cores;
    bool governor_perf{false};
    std::string saved_gpu_gov{"simple_ondemand"};
    std::string saved_npu_gov{"rknpu_ondemand"};
    bool gpu_gov_boosted{false};
    bool npu_gov_boosted{false};

    std::vector<std::thread> cpu_threads;
    pid_t gpu_pid{0};
    std::vector<pid_t> npu_pids;
    pid_t rga_pid{0};
    pid_t mpp_pid{0};

    // MPP 硬件中断帧率采样基线
    uint64_t prev_mpp_enc_irqs{0};
    uint64_t prev_mpp_dec_irqs{0};
    uint64_t prev_mpp_sample_us{0};
    uint64_t mpp_base_enc_irqs{0};
    uint64_t mpp_base_dec_irqs{0};
    bool mpp_irq_initialized{false};
    double smooth_mpp_enc_fps{0.0};
    double smooth_mpp_dec_fps{0.0};

    // 大容量 DDR 带宽与位翻转完整性校验压测
    DdrStressTester ddr_tester;

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
    void set_keep_isolcpus(bool keep);
    int get_active_cpu_burn_cores() const;

    void toggle_cpu_stress();
    std::string toggle_gpu_stress();
    std::string toggle_npu_stress();
    std::string toggle_rga_stress();
    std::string toggle_mpp_stress();
    std::string toggle_ddr_stress();
    std::string toggle_all_stress();
    std::string check_stress_health();
    void stop_all_stress();
    void toggle_governor();

    std::pair<double, std::vector<double>> get_cpu_loads();
    std::vector<int> get_cpu_freqs() const;
    std::pair<double, int> get_gpu_stats() const;
    std::pair<std::vector<double>, int> get_npu_stats() const;
    std::pair<std::vector<double>, int> get_rga_stats() const;
    MppStats get_mpp_stats();
    std::pair<uint64_t, uint64_t> get_mpp_session_frames() const;
    std::map<std::string, double> get_thermals() const;
    ThermalThrottleState get_throttle_state() const;
    std::tuple<double, double, double> get_mem_stats() const;
    std::string get_uptime() const;

private:
    void init_governor_state();
    void init_thermals();
    void init_cooling_devices();
    void rebuild_cpu_burn_list();
};
