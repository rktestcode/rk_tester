#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct SentSlot {
    uint64_t send_time_us{0};
    uint16_t seq{0};
    bool active{false};
    bool replied{false};
    bool marked_lost{false};
};

struct NetPortInfo {
    static constexpr size_t kSentRingSize = 2048;

    std::string name;       // lan0, lan1, lan2, lan3
    std::string ch_name;    // Port 1 (lan0->lan1)
    std::string peer_name;  // lan1
    std::string local_ip;   // 192.168.10.1
    std::string peer_ip;    // 192.168.11.1
    std::string current_ip; // Current IP
    uint32_t local_ip_be{0};
    uint32_t peer_ip_be{0};
    int peer_idx{-1};       // Paired port index in net_ports
    int ifindex{-1};
    uint8_t mac[6]{0};
    uint8_t peer_mac[6]{0};
    bool has_peer_mac{false};
    bool carrier{false};          // Link carrier up/down
    bool carrier_initialized{false};
    int speed{-1};                // Speed (1000, 100, 10)

    // Per-port independent AF_PACKET raw socket and stats
    int raw_sock{-1};
    uint16_t ping_id{0};
    uint64_t sent{0};
    uint64_t recv{0};
    uint64_t lost{0};
    uint64_t flaps{0};
    uint64_t tx_bytes{0};
    uint64_t rx_bytes{0};
    uint64_t tx_bytes_win{0};
    uint64_t rx_bytes_win{0};
    double tx_mbps{0.0};
    double rx_mbps{0.0};
    double rtt_cur{0.0};
    double rtt_min{999999.0};
    double rtt_max{0.0};
    double rtt_sum{0.0};
    uint64_t hw_rx_err_base{0};
    uint64_t hw_tx_err_base{0};
    uint64_t hw_rx_drop_base{0};
    bool hw_err_inited{false};
    uint64_t hw_rx_err{0};
    uint64_t hw_tx_err{0};
    uint64_t hw_rx_drop{0};
    uint64_t last_drop_log_us{0};
    uint64_t last_slow_tx_us{0};
    std::array<SentSlot, kSentRingSize> sent_ring{};
    size_t inflight_cnt{0};
    bool enabled{true};

    NetPortInfo() = default;
    NetPortInfo(std::string n, std::string ch, std::string peer, int pidx, std::string lip, std::string pip);
};

// 只读快照，用于主线程 TUI 与报告安全读取
struct NetPortSnapshot {
    std::string name;
    std::string ch_name;
    std::string current_ip;
    bool carrier{false};
    int speed{-1};
    bool enabled{true};
    uint64_t sent{0};
    uint64_t recv{0};
    uint64_t lost{0};
    uint64_t flaps{0};
    uint64_t tx_bytes{0};
    uint64_t rx_bytes{0};
    double tx_mbps{0.0};
    double rx_mbps{0.0};
    double rtt_cur{0.0};
    double rtt_min{999999.0};
    double rtt_max{0.0};
    double rtt_sum{0.0};
    uint64_t hw_rx_err{0};
    uint64_t hw_tx_err{0};
    uint64_t hw_rx_drop{0};
};

struct EmcSnapshot {
    std::vector<NetPortSnapshot> ports;
    std::string latest_event;
    int latest_event_color{1};
    bool high_load{false};
    uint64_t total_sent{0};
    uint64_t total_recv{0};
    uint64_t total_lost{0};
    uint64_t total_flaps{0};
    uint64_t total_tx_bytes{0};
    uint64_t total_rx_bytes{0};
    double total_tx_mbps{0.0};
    double total_rx_mbps{0.0};
};

class NetEmcTester {
public:
    std::atomic<bool> active{false};
    std::atomic<bool> high_load{false}; // false: 100ms EMC监测 | true: 1480B 高吞吐千兆打流
    int interval_ms{100};               // EMC 监测模式发包周期
    int packet_size{1000};              // EMC 监测模式包长
    std::string log_path;

    NetEmcTester();
    ~NetEmcTester();

    EmcSnapshot get_snapshot() const;
    void set_port_enabled(size_t idx, bool en);
    bool get_port_enabled(size_t idx) const;
    void toggle_high_load();
    void set_high_load(bool en);
    void reset_stats();
    void log_event(const std::string& level, const std::string& msg, int color);
    void setup_default_network();

private:
    static constexpr int kBurstBatch = 16;
    static constexpr int kRecvBatch = 32;

    mutable std::mutex mtx;
    std::vector<NetPortInfo> net_ports;
    std::string latest_event{"EMC test on STANDBY. Press [5] to start monitoring."};
    int latest_event_color{1};
    std::ofstream log_file;
    std::atomic<bool> thread_stop{false};
    std::atomic<bool> reinit_req{false};
    int wake_pipe[2]{-1, -1};
    std::thread worker;
    uint8_t tx_batch_buf[kBurstBatch][1536]{};
    uint8_t rx_batch_buf[kRecvBatch][2048]{};

    void init_net_ports();
    static bool is_nonzero_mac(const uint8_t m[6]);
    static bool read_iface_mac(const std::string& name, uint8_t out_mac[6]);
    static int open_raw_packet_socket(int ifindex);

    void refresh_peer_macs_locked();
    void rebind_all_net_sockets_locked();
    void init_emc();
    void log_event_locked(const std::string& level, const std::string& msg, int color);
    void update_net_status();
    void tick_send_locked();
    void tick_recv_locked();
    void tick_timeout_locked();
    void thread_loop();
};
