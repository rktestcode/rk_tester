#pragma once

#include "common.hpp"

#include <atomic>
#include <cstdint>
#include <deque>
#include <linux/can.h>
#include <mutex>
#include <string>
#include <thread>

enum class CanDirection {
    CAN0_TO_CAN1 = 0,   // CAN0 TX -> CAN1 RX (Uni) [Default recommended]
    CAN1_TO_CAN0 = 1,   // CAN1 TX -> CAN0 RX (Uni)
    PING_PONG = 2       // CAN0 <-> CAN1 (Ping-Pong Request/Response)
};

class CanBusTester {
public:
    std::string if0{"can0"};
    std::string if1{"can1"};
    std::atomic<bool> active{false};
    std::atomic<ChannelState> state{ChannelState::STANDBY};
    std::atomic<CanDirection> dir{CanDirection::CAN0_TO_CAN1};

    std::atomic<uint64_t> tx_bytes{0};
    std::atomic<uint64_t> rx_bytes{0};
    std::atomic<uint64_t> tx_frames{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> err_frames{0};
    std::atomic<uint64_t> crc_errors{0};
    std::atomic<uint64_t> bus_errors{0};

    std::atomic<double> tx_fps{0.0};
    std::atomic<double> rx_fps{0.0};
    std::atomic<double> rtt_cur_ms{0.0};
    std::atomic<double> rtt_max_ms{0.0};
    std::atomic<double> rtt_sum_ms{0.0};
    std::atomic<uint8_t> last_tx_seq{0};
    std::atomic<uint8_t> last_rx_seq{0};
    std::atomic<uint64_t> last_valid_rx_us{0};

    std::thread worker;

    CanBusTester() = default;
    ~CanBusTester();

    void cycle_direction();
    void set_direction(CanDirection d);

    std::string get_short_label() const;
    std::string get_wide_label() const;
    std::string get_mode_name() const;
    std::string get_short_mode_name() const;
    std::string get_menu_desc() const;
    std::string get_error_detail() const;
    std::string get_bitrate_info() const;

    void start();
    void stop();
    void reset();

private:
    struct PendingCanFrame {
        uint8_t seq{0};
        uint64_t send_time_us{0};
        uint8_t expect_data[8]{};
    };

    std::atomic<bool> reset_requested{false};
    mutable std::mutex meta_mtx;
    std::string last_error_detail;
    std::string bitrate_info{"500k"};

    void set_error_detail(const std::string& msg);
    void set_bitrate_info(const std::string& info);
    void clear_counters();

    static bool is_iface_up(const std::string& dev);
    static std::string ensure_can_up(const std::string& dev);
    static void flush_can_socket(int s);
    static int open_socket(const std::string& ifname, canid_t filter_id, int& out_errno);

    void process_rx(int s0, int s1, CanDirection cur_dir,
                    std::deque<PendingCanFrame>& pending_tx,
                    uint64_t& rx_calc);
    void run();
};
