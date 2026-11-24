#pragma once

#include "common.hpp"

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct SerialFrame {
    uint8_t sync1{0xAA};
    uint8_t sync2{0x55};
    uint16_t seq{0};
    uint8_t payload[58];
    uint16_t checksum{0};
} __attribute__((packed));

enum class SerialDirection {
    A_TO_B = 0,     // Port A TX -> Port B RX (Uni)
    B_TO_A = 1,     // Port B TX -> Port A RX (Uni)
    PING_PONG = 2   // Port A <-> Port B (Ping-Pong Request/Response)
};

class SerialChannel {
public:
    std::string proto_name; // "RS232" / "RS485"
    std::string short_a;    // "S0" / "S2"
    std::string short_b;    // "S1" / "S3"
    std::string tx_dev;     // "/dev/ttyS0" / "/dev/ttyS2" (Port A)
    std::string rx_dev;     // "/dev/ttyS1" / "/dev/ttyS3" (Port B)
    std::string dir_key;    // "Z" / "X"
    std::atomic<bool> active{false};
    std::atomic<ChannelState> state{ChannelState::STANDBY};
    std::atomic<SerialDirection> dir{SerialDirection::A_TO_B};

    std::atomic<uint64_t> tx_bytes{0};
    std::atomic<uint64_t> rx_bytes{0};
    std::atomic<uint64_t> tx_frames{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> err_frames{0};
    std::atomic<uint64_t> crc_errors{0};
    std::atomic<uint64_t> hw_oe{0};
    std::atomic<uint64_t> hw_fe{0};
    std::atomic<uint64_t> hw_pe{0};
    std::atomic<double> speed_kb{0.0};
    std::atomic<double> tx_speed_kb{0.0};
    std::atomic<double> tx_fps{0.0};
    std::atomic<double> rx_fps{0.0};
    std::atomic<double> rtt_cur_ms{0.0};
    std::atomic<double> rtt_max_ms{0.0};
    std::atomic<double> rtt_sum_ms{0.0};
    std::atomic<uint16_t> last_tx_seq{0};
    std::atomic<uint16_t> last_rx_seq{0};
    std::atomic<uint64_t> last_valid_rx_us{0};

    std::thread worker;

    SerialChannel(const std::string& proto, const std::string& sa, const std::string& sb,
                  const std::string& dev_a, const std::string& dev_b, const std::string& key);
    ~SerialChannel();

    void cycle_direction();
    void set_direction(SerialDirection d);

    std::string get_short_label() const;
    std::string get_wide_label() const;
    std::string get_short_mode_name() const;
    std::string get_mode_name() const;
    std::string get_menu_desc() const;
    std::string get_error_detail() const;

    void start();
    void request_stop() { active.store(false); }
    void stop();
    void reset();

private:
    struct PendingSerialFrame {
        uint16_t seq{0};
        uint64_t send_time_us{0};
        uint8_t expect_payload[58]{};
    };

    std::atomic<bool> reset_requested{false};
    mutable std::mutex err_mtx;
    std::string last_error_detail;

    void set_error_detail(const std::string& msg);
    void clear_counters();

    static int open_port(const std::string& path, int& out_errno);
    static bool write_all_nonblock(int fd, const uint8_t* data, size_t len);
    static SerialFrame build_frame_from_payload(uint8_t s1, uint8_t s2, uint16_t seq, const uint8_t payload[58]);
    static SerialFrame build_frame(uint8_t s1, uint8_t s2, uint16_t seq, uint8_t salt);
    static void drain_fd_into(int fd, std::vector<uint8_t>& buf, std::atomic<uint64_t>* byte_counter);
    static void discard_fd(int fd);
    static bool read_uart_hw_errors(const std::string& dev_path, uint64_t& oe, uint64_t& fe, uint64_t& pe);

    void match_rx_stream(std::vector<uint8_t>& rx_buf, uint8_t exp_s1, uint8_t exp_s2,
                         std::deque<PendingSerialFrame>& pending_tx,
                         uint64_t& rx_bytes_since_calc, uint64_t& rx_frames_since_calc);

    void process_rx(int fd_a, int fd_b, SerialDirection cur_dir,
                    std::vector<uint8_t>& rx_buf_a, std::vector<uint8_t>& rx_buf_b,
                    std::deque<PendingSerialFrame>& pending_tx,
                    uint64_t& rx_bytes_since_calc, uint64_t& rx_frames_since_calc);

    void run();
};
