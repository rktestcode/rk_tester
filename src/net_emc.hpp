#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct SentPacket {
    uint64_t send_time_us{0};
    bool replied{false};
    bool marked_lost{false};
};

struct NetPortInfo {
    std::string name;       // eth0, eth1, eth2, eth3
    std::string ch_name;    // Port 1 (eth0->eth1)
    std::string peer_name;  // eth1
    std::string local_ip;   // 192.168.10.1
    std::string peer_ip;    // 192.168.11.1
    std::string current_ip; // Current IP
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
    double rtt_cur{0.0};
    double rtt_min{999999.0};
    double rtt_max{0.0};
    double rtt_sum{0.0};
    std::map<uint16_t, SentPacket> sent_map;
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
    double rtt_cur{0.0};
    double rtt_min{999999.0};
    double rtt_max{0.0};
    double rtt_sum{0.0};
};

struct EmcSnapshot {
    std::vector<NetPortSnapshot> ports;
    std::string latest_event;
    int latest_event_color{1};
    uint64_t total_sent{0};
    uint64_t total_recv{0};
    uint64_t total_lost{0};
    uint64_t total_flaps{0};
};

class NetEmcTester {
public:
    std::atomic<bool> active{false};
    int interval_ms{100};  // 100ms 高频发包
    int packet_size{1000}; // 1000 字节大包
    std::string log_path;

    NetEmcTester();
    ~NetEmcTester();

    EmcSnapshot get_snapshot() const;
    void set_port_enabled(size_t idx, bool en);
    bool get_port_enabled(size_t idx) const;
    void reset_stats();
    void log_event(const std::string& level, const std::string& msg, int color);
    void setup_default_network();

private:
    mutable std::mutex mtx;
    std::vector<NetPortInfo> net_ports;
    std::string latest_event{"EMC test on STANDBY. Press [5] to start monitoring."};
    int latest_event_color{1};
    std::ofstream log_file;
    std::atomic<bool> thread_stop{false};
    std::atomic<bool> reinit_req{false};
    std::thread worker;

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
