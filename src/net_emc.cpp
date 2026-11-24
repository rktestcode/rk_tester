#include "net_emc.hpp"
#include "common.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iomanip>
#include <linux/filter.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <netpacket/packet.h>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace {

uint64_t read_sys_u64(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return 0;
    unsigned long long v = 0;
    if (std::fscanf(f, "%llu", &v) != 1) v = 0;
    std::fclose(f);
    return static_cast<uint64_t>(v);
}

int read_sys_int(const std::string& path, int def_val) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return def_val;
    int v = def_val;
    if (std::fscanf(f, "%d", &v) != 1) v = def_val;
    std::fclose(f);
    return v;
}

} // namespace

NetPortInfo::NetPortInfo(std::string n, std::string ch, std::string peer, int pidx, std::string lip, std::string pip)
    : name(std::move(n)), ch_name(std::move(ch)), peer_name(std::move(peer)),
      local_ip(lip), peer_ip(pip), current_ip(lip),
      local_ip_be(inet_addr(lip.c_str())), peer_ip_be(inet_addr(pip.c_str())),
      peer_idx(pidx) {}

NetEmcTester::NetEmcTester() {
    if (pipe2(wake_pipe, O_NONBLOCK | O_CLOEXEC) != 0) {
        wake_pipe[0] = -1;
        wake_pipe[1] = -1;
    }
    init_net_ports();
    setup_default_network();
    init_emc();
    update_net_status();

    thread_stop.store(false);
    worker = std::thread(&NetEmcTester::thread_loop, this);
}

NetEmcTester::~NetEmcTester() {
    active.store(false);
    thread_stop.store(true);
    if (wake_pipe[1] >= 0) {
        uint8_t b = 1;
        if (write(wake_pipe[1], &b, 1) < 0) {}
    }
    if (worker.joinable()) worker.join();

    if (wake_pipe[0] >= 0) { close(wake_pipe[0]); wake_pipe[0] = -1; }
    if (wake_pipe[1] >= 0) { close(wake_pipe[1]); wake_pipe[1] = -1; }
    std::lock_guard<std::mutex> lk(mtx);
    for (auto& port : net_ports) {
        if (port.raw_sock >= 0) {
            close(port.raw_sock);
            port.raw_sock = -1;
        }
    }
    if (log_file.is_open()) log_file.close();
}

EmcSnapshot NetEmcTester::get_snapshot() const {
    std::lock_guard<std::mutex> lk(mtx);
    EmcSnapshot snap;
    snap.latest_event = latest_event;
    snap.latest_event_color = latest_event_color;
    snap.high_load = high_load.load();
    snap.ports.reserve(net_ports.size());
    for (const auto& p : net_ports) {
        NetPortSnapshot ps;
        ps.name = p.name;
        ps.ch_name = p.ch_name;
        ps.current_ip = p.current_ip;
        ps.carrier = p.carrier;
        ps.speed = p.speed;
        ps.enabled = p.enabled;
        ps.sent = p.sent;
        ps.recv = p.recv;
        ps.lost = p.lost;
        ps.flaps = p.flaps;
        ps.tx_bytes = p.tx_bytes;
        ps.rx_bytes = p.rx_bytes;
        ps.tx_mbps = p.tx_mbps;
        ps.rx_mbps = p.rx_mbps;
        ps.rtt_cur = p.rtt_cur;
        ps.rtt_min = p.rtt_min;
        ps.rtt_max = p.rtt_max;
        ps.rtt_sum = p.rtt_sum;
        ps.hw_rx_err = p.hw_rx_err;
        ps.hw_tx_err = p.hw_tx_err;
        ps.hw_rx_drop = p.hw_rx_drop;
        snap.total_sent += p.sent;
        snap.total_recv += p.recv;
        snap.total_lost += p.lost;
        snap.total_flaps += p.flaps;
        snap.total_tx_bytes += p.tx_bytes;
        snap.total_rx_bytes += p.rx_bytes;
        snap.total_tx_mbps += p.tx_mbps;
        snap.total_rx_mbps += p.rx_mbps;
        snap.ports.push_back(std::move(ps));
    }
    return snap;
}

void NetEmcTester::set_port_enabled(size_t idx, bool en) {
    std::lock_guard<std::mutex> lk(mtx);
    if (idx < net_ports.size()) {
        net_ports[idx].enabled = en;
    }
}

bool NetEmcTester::get_port_enabled(size_t idx) const {
    std::lock_guard<std::mutex> lk(mtx);
    return (idx < net_ports.size()) ? net_ports[idx].enabled : true;
}

void NetEmcTester::toggle_high_load() {
    set_high_load(!high_load.load());
}

void NetEmcTester::set_high_load(bool en) {
    high_load.store(en);
    std::lock_guard<std::mutex> lk(mtx);
    for (auto& p : net_ports) {
        p.tx_bytes_win = 0;
        p.rx_bytes_win = 0;
        p.tx_mbps = 0.0;
        p.rx_mbps = 0.0;
    }
}

void NetEmcTester::reset_stats() {
    std::lock_guard<std::mutex> lk(mtx);
    for (auto& p : net_ports) {
        p.sent = 0;
        p.recv = 0;
        p.lost = 0;
        p.flaps = 0;
        p.tx_bytes = 0;
        p.rx_bytes = 0;
        p.tx_bytes_win = 0;
        p.rx_bytes_win = 0;
        p.tx_mbps = 0.0;
        p.rx_mbps = 0.0;
        p.rtt_cur = 0.0;
        p.rtt_min = 999999.0;
        p.rtt_max = 0.0;
        p.rtt_sum = 0.0;
        p.hw_err_inited = false;
        p.hw_rx_err = 0;
        p.hw_tx_err = 0;
        p.hw_rx_drop = 0;
        p.last_drop_log_us = 0;
        p.last_slow_tx_us = 0;
        p.sent_ring.fill(SentSlot{});
        p.inflight_cnt = 0;
    }
}

void NetEmcTester::log_event(const std::string& level, const std::string& msg, int color) {
    std::lock_guard<std::mutex> lk(mtx);
    log_event_locked(level, msg, color);
}

void NetEmcTester::setup_default_network() {
    struct NetCfg {
        const char* iface;
        const char* ip;
        const char* brd;
    } cfgs[] = {
        {"lan0", "192.168.10.1", "192.168.10.255"},
        {"lan1", "192.168.11.1", "192.168.11.255"},
        {"lan2", "192.168.20.1", "192.168.20.255"},
        {"lan3", "192.168.21.1", "192.168.21.255"}
    };

    for (const auto& c : cfgs) {
        std::string up_cmd = std::string("ip link set dev ") + c.iface + " up 2>/dev/null";
        if (system(up_cmd.c_str()) < 0) {}

        std::string chk_cmd = std::string("ip addr show dev ") + c.iface + " 2>/dev/null | grep 'inet " + c.ip + "/' | grep -q 'brd '";
        if (system(chk_cmd.c_str()) != 0) {
            std::string del_cmd = std::string("ip addr del ") + c.ip + "/24 dev " + c.iface + " 2>/dev/null";
            if (system(del_cmd.c_str()) < 0) {}
            std::string add_cmd = std::string("ip addr add ") + c.ip + "/24 brd " + c.brd + " dev " + c.iface + " 2>/dev/null";
            if (system(add_cmd.c_str()) < 0) {}
        }
    }
    reinit_req.store(true);
}

void NetEmcTester::init_net_ports() {
    net_ports.clear();
    net_ports.emplace_back("lan0", "Port 1 (lan0->lan1)", "lan1", 1, "192.168.10.1", "192.168.11.1");
    net_ports.emplace_back("lan1", "Port 2 (lan1->lan0)", "lan0", 0, "192.168.11.1", "192.168.10.1");
    net_ports.emplace_back("lan2", "Port 3 (lan2->lan3)", "lan3", 3, "192.168.20.1", "192.168.21.1");
    net_ports.emplace_back("lan3", "Port 4 (lan3->lan2)", "lan2", 2, "192.168.21.1", "192.168.20.1");
}

bool NetEmcTester::is_nonzero_mac(const uint8_t m[6]) {
    for (int i = 0; i < 6; ++i) {
        if (m[i] != 0) return true;
    }
    return false;
}

bool NetEmcTester::read_iface_mac(const std::string& name, uint8_t out_mac[6]) {
    std::ifstream fmac("/sys/class/net/" + name + "/address");
    if (!fmac.is_open()) return false;
    std::string mac_str;
    fmac >> mac_str;
    unsigned int m[6];
    if (sscanf(mac_str.c_str(), "%x:%x:%x:%x:%x:%x",
               &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
        for (int k = 0; k < 6; ++k) out_mac[k] = static_cast<uint8_t>(m[k]);
        return true;
    }
    return false;
}

int NetEmcTester::open_raw_packet_socket(int ifindex) {
    if (ifindex <= 0) return -1;
    int s = socket(AF_PACKET, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, htons(ETH_P_IP));
    if (s < 0) return -1;

    // 内核态 BPF 过滤：丢弃自身发出的 PACKET_OUTGOING 以及非 IPv4 ICMP 流量（如管理口 SSH TCP），
    // 避免原始套接字被无关流量高频唤醒，大幅降低 CPU 空耗
    struct sock_filter icmp_filter[] = {
        BPF_STMT(BPF_LD | BPF_W | BPF_ABS, static_cast<uint32_t>(SKF_AD_OFF + SKF_AD_PKTTYPE)),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, PACKET_OUTGOING, 5, 0),
        BPF_STMT(BPF_LD | BPF_H | BPF_ABS, 12),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, ETH_P_IP, 0, 3),
        BPF_STMT(BPF_LD | BPF_B | BPF_ABS, 23),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, IPPROTO_ICMP, 0, 1),
        BPF_STMT(BPF_RET | BPF_K, 0xFFFF),
        BPF_STMT(BPF_RET | BPF_K, 0),
    };
    struct sock_fprog bpf_prog{};
    bpf_prog.len = static_cast<unsigned short>(sizeof(icmp_filter) / sizeof(icmp_filter[0]));
    bpf_prog.filter = icmp_filter;
    setsockopt(s, SOL_SOCKET, SO_ATTACH_FILTER, &bpf_prog, sizeof(bpf_prog));

    // 增大收发内核缓冲至 4MB（root 下优先使用 SO_RCVBUFFORCE/SO_SNDBUFFORCE 绕过默认 208KB rmem_max 上限）
    int buf_sz = 4 * 1024 * 1024;
#ifdef SO_RCVBUFFORCE
    if (setsockopt(s, SOL_SOCKET, SO_RCVBUFFORCE, &buf_sz, sizeof(buf_sz)) != 0)
#endif
    {
        setsockopt(s, SOL_SOCKET, SO_RCVBUF, &buf_sz, sizeof(buf_sz));
    }
#ifdef SO_SNDBUFFORCE
    if (setsockopt(s, SOL_SOCKET, SO_SNDBUFFORCE, &buf_sz, sizeof(buf_sz)) != 0)
#endif
    {
        setsockopt(s, SOL_SOCKET, SO_SNDBUF, &buf_sz, sizeof(buf_sz));
    }

    struct sockaddr_ll sll{};
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifindex;
    sll.sll_protocol = htons(ETH_P_IP);
    if (bind(s, reinterpret_cast<struct sockaddr*>(&sll), sizeof(sll)) < 0) {
        close(s);
        return -1;
    }
    return s;
}

void NetEmcTester::refresh_peer_macs_locked() {
    for (auto& port : net_ports) {
        if (port.peer_idx >= 0 && port.peer_idx < static_cast<int>(net_ports.size())) {
            const auto& peer = net_ports[port.peer_idx];
            if (is_nonzero_mac(peer.mac)) {
                std::memcpy(port.peer_mac, peer.mac, 6);
                port.has_peer_mac = true;
            } else {
                port.has_peer_mac = false;
            }
        }
    }
}

void NetEmcTester::rebind_all_net_sockets_locked() {
    uint16_t base_id = static_cast<uint16_t>(getpid() & 0x3FFF);
    for (size_t i = 0; i < net_ports.size(); ++i) {
        auto& port = net_ports[i];
        port.ifindex = static_cast<int>(if_nametoindex(port.name.c_str()));
        read_iface_mac(port.name, port.mac);
        port.ping_id = static_cast<uint16_t>(base_id + i * 1000 + 100);
        if (port.raw_sock >= 0) {
            close(port.raw_sock);
            port.raw_sock = -1;
        }
        if (port.ifindex > 0) {
            port.raw_sock = open_raw_packet_socket(port.ifindex);
        }
    }
    refresh_peer_macs_locked();
}

void NetEmcTester::init_emc() {
    {
        std::lock_guard<std::mutex> lk(mtx);
        rebind_all_net_sockets_locked();
    }
    reinit_req.store(false);

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_buf{};
    localtime_r(&in_time_t, &tm_buf);
    std::stringstream ss;
    ss << "/tmp/emc_test_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".log";
    log_path = ss.str();
    log_file.open(log_path, std::ios::out | std::ios::app);
}

void NetEmcTester::log_event_locked(const std::string& level, const std::string& msg, int color) {
    std::string ts = get_time_str();
    latest_event = "[" + ts + "] " + msg;
    latest_event_color = color;
    if (log_file.is_open()) {
        log_file << "[" << ts << "] [" << level << "] " << msg << std::endl;
        log_file.flush();
    }
}

void NetEmcTester::update_net_status() {
    // 先无锁读取 sysfs，避免文件 I/O 占用互斥锁
    struct SysNetState {
        bool carrier{false};
        int speed{-1};
        int ifindex{-1};
        uint8_t mac[6]{0};
        bool has_mac{false};
        uint64_t rx_err{0};
        uint64_t tx_err{0};
        uint64_t rx_drop{0};
    };
    std::vector<std::string> names;
    {
        std::lock_guard<std::mutex> lk(mtx);
        names.reserve(net_ports.size());
        for (const auto& p : net_ports) names.push_back(p.name);
    }

    std::vector<SysNetState> states(names.size());
    for (size_t i = 0; i < names.size(); ++i) {
        const auto& iface = names[i];
        std::string base = "/sys/class/net/" + iface;
        states[i].carrier = (read_sys_int(base + "/carrier", 0) == 1);
        states[i].speed = read_sys_int(base + "/speed", -1);
        states[i].ifindex = static_cast<int>(if_nametoindex(iface.c_str()));
        states[i].has_mac = read_iface_mac(iface, states[i].mac);
        states[i].rx_err = read_sys_u64(base + "/statistics/rx_errors");
        states[i].tx_err = read_sys_u64(base + "/statistics/tx_errors");
        states[i].rx_drop = read_sys_u64(base + "/statistics/rx_dropped");
    }

    std::lock_guard<std::mutex> lk(mtx);
    bool mac_updated = false;
    for (size_t i = 0; i < net_ports.size() && i < states.size(); ++i) {
        auto& port = net_ports[i];
        const auto& st = states[i];

        port.current_ip = port.local_ip;
        port.speed = st.speed; // 先更新速率，确保 Link Up 日志打印真实协商速率
        if (!port.hw_err_inited) {
            port.hw_rx_err_base = st.rx_err;
            port.hw_tx_err_base = st.tx_err;
            port.hw_rx_drop_base = st.rx_drop;
            port.hw_err_inited = true;
        }
        port.hw_rx_err = (st.rx_err >= port.hw_rx_err_base) ? (st.rx_err - port.hw_rx_err_base) : 0;
        port.hw_tx_err = (st.tx_err >= port.hw_tx_err_base) ? (st.tx_err - port.hw_tx_err_base) : 0;
        port.hw_rx_drop = (st.rx_drop >= port.hw_rx_drop_base) ? (st.rx_drop - port.hw_rx_drop_base) : 0;

        if (st.has_mac && std::memcmp(port.mac, st.mac, 6) != 0) {
            std::memcpy(port.mac, st.mac, 6);
            mac_updated = true;
        }

        if (st.ifindex > 0 && st.ifindex != port.ifindex) {
            port.ifindex = st.ifindex;
            if (port.raw_sock >= 0) close(port.raw_sock);
            port.raw_sock = open_raw_packet_socket(port.ifindex);
        }

        if (!port.carrier_initialized) {
            port.carrier = st.carrier;
            port.carrier_initialized = true;
        } else if (st.carrier != port.carrier) {
            if (port.carrier && !st.carrier) {
                port.flaps++;
                log_event_locked("FLAP", "CRITICAL: " + port.ch_name + " Link Down (cable unplugged)!", 3);
            } else if (!port.carrier && st.carrier) {
                std::string spd_str = (port.speed > 0) ? (std::to_string(port.speed) + "M") : "UP";
                log_event_locked("INFO", "INFO: " + port.ch_name + " Link Up (" + spd_str + ")! Auto-resuming ping.", 1);
                if (port.raw_sock < 0 && port.ifindex > 0) {
                    port.raw_sock = open_raw_packet_socket(port.ifindex);
                }
            }
            port.carrier = st.carrier;
        }
    }
    if (mac_updated) refresh_peer_macs_locked();
}

void NetEmcTester::tick_send_locked() {
    if (!active.load()) return;

    const bool hl = high_load.load();
    const uint64_t now_us = get_time_us();

    for (size_t idx = 0; idx < net_ports.size(); ++idx) {
        auto& port = net_ports[idx];
        if (!port.enabled) continue;

        bool peer_up = (port.peer_idx >= 0 && port.peer_idx < static_cast<int>(net_ports.size()) &&
                        net_ports[static_cast<size_t>(port.peer_idx)].enabled &&
                        net_ports[static_cast<size_t>(port.peer_idx)].carrier);
        // 安全保护：仅当本口与本机对端口均启用且 Link UP 时才启用高吞吐突发打流，
        // 避免单口接入外部交换机（如 SSH 管理口 lan2，对端 lan3 未接）时泛洪办公网
        bool port_burst = hl && port.carrier && peer_up && port.raw_sock >= 0 && port.ifindex > 0;

        int batch = 1;
        if (!port_burst) {
            if (now_us < port.last_slow_tx_us + static_cast<uint64_t>(interval_ms) * 1000ULL) {
                continue;
            }
            port.last_slow_tx_us = now_us;
        } else {
            // 闭环在途窗口自时钟流控（按物理协商速率自适应：1000M->48帧，100M->12帧，10M->4帧），
            // 既能打满物理线速（100M/1000M），又彻底杜绝开环发包撑爆网卡/socket缓冲导致的虚假丢包与高延迟
            size_t max_win = (port.speed >= 1000) ? 48 : ((port.speed >= 100) ? 12 : 4);
            if (port.inflight_cnt >= max_win) {
                continue;
            }
            size_t room = max_win - port.inflight_cnt;
            batch = static_cast<int>(std::min<size_t>(static_cast<size_t>(kBurstBatch), room));
        }

        if (!port.carrier || port.raw_sock < 0 || port.ifindex <= 0) {
            port.sent++;
            port.lost++;
            continue;
        }

        const int pkt_len = port_burst ? 1480 : std::min(1536, std::max(64, packet_size));

        struct sockaddr_ll sll{};
        sll.sll_family = AF_PACKET;
        sll.sll_ifindex = port.ifindex;
        sll.sll_protocol = htons(ETH_P_IP);

        struct mmsghdr msgs[kBurstBatch]{};
        struct iovec iovs[kBurstBatch]{};
        uint16_t seqs[kBurstBatch]{};

        for (int b = 0; b < batch; ++b) {
            uint8_t* pkt = tx_batch_buf[b];
            // 仅清零包头与负载关键区，高吞吐模式下避免大块重复 memset
            std::memset(pkt, 0, 64);
            if (!port_burst && pkt_len > 64) {
                std::memset(pkt + 64, 0x5A, static_cast<size_t>(pkt_len - 64));
            } else if (pkt_len > 64) {
                // 保持非零固定负载提高 PHY 信号切换率
                pkt[64] = static_cast<uint8_t>(0xA5 ^ b);
                pkt[pkt_len - 1] = static_cast<uint8_t>(0x5A ^ b);
            }

            auto* eth = reinterpret_cast<struct ether_header*>(pkt);
            if (port.has_peer_mac) {
                std::memcpy(eth->ether_dhost, port.peer_mac, 6);
            } else {
                std::memset(eth->ether_dhost, 0xff, 6);
            }
            std::memcpy(eth->ether_shost, port.mac, 6);
            eth->ether_type = htons(ETHERTYPE_IP);

            uint16_t seq = static_cast<uint16_t>((port.sent + static_cast<uint64_t>(b)) & 0xFFFFu);
            seqs[b] = seq;

            auto* ip = reinterpret_cast<struct iphdr*>(pkt + 14);
            ip->version = 4;
            ip->ihl = 5;
            ip->tot_len = htons(static_cast<uint16_t>(pkt_len - 14));
            ip->id = htons(seq);
            ip->ttl = 64;
            ip->protocol = IPPROTO_ICMP;
            ip->saddr = port.local_ip_be;
            ip->daddr = port.peer_ip_be;
            ip->check = in_cksum(ip, 20);

            auto* icmp = reinterpret_cast<struct icmphdr*>(pkt + 34);
            icmp->type = ICMP_ECHO;
            icmp->code = 0;
            icmp->un.echo.id = htons(port.ping_id);
            icmp->un.echo.sequence = htons(seq);

            std::memcpy(pkt + 42, "RK_NET_PING", 11);
            std::memcpy(pkt + 54, &now_us, sizeof(now_us));
            // 高吞吐模式下仅校验 ICMP 头 + 时间戳签名（对端直接反射，保持高带宽零 CPU 瓶颈）
            icmp->checksum = port_burst ? in_cksum(icmp, 32) : in_cksum(icmp, pkt_len - 34);

            iovs[b].iov_base = pkt;
            iovs[b].iov_len = static_cast<size_t>(pkt_len);
            msgs[b].msg_hdr.msg_name = &sll;
            msgs[b].msg_hdr.msg_namelen = sizeof(sll);
            msgs[b].msg_hdr.msg_iov = &iovs[b];
            msgs[b].msg_hdr.msg_iovlen = 1;
        }

        int n_sent = sendmmsg(port.raw_sock, msgs, static_cast<unsigned int>(batch), MSG_DONTWAIT);
        if (n_sent < 0) n_sent = 0;

        for (int b = 0; b < n_sent; ++b) {
            port.sent++;
            port.tx_bytes += static_cast<uint64_t>(pkt_len);
            port.tx_bytes_win += static_cast<uint64_t>(pkt_len);
            uint16_t seq = seqs[b];
            auto& slot = port.sent_ring[seq & (NetPortInfo::kSentRingSize - 1)];
            if (slot.active && !slot.replied && !slot.marked_lost) {
                port.lost++;
            } else if (!slot.active) {
                port.inflight_cnt++;
            }
            slot.send_time_us = now_us;
            slot.seq = seq;
            slot.active = true;
            slot.replied = false;
            slot.marked_lost = false;
        }
        if (!port_burst && n_sent == 0) {
            port.sent++;
            port.lost++;
        }
    }
}

void NetEmcTester::tick_recv_locked() {
    struct mmsghdr rmsgs[kRecvBatch]{};
    struct iovec riovs[kRecvBatch]{};
    struct sockaddr_ll raddrs[kRecvBatch]{};

    struct mmsghdr smsgs[kRecvBatch]{};
    struct iovec siovs[kRecvBatch]{};

    for (auto& port : net_ports) {
        if (port.raw_sock < 0 || port.ifindex <= 0) continue;

        struct sockaddr_ll reply_sll{};
        reply_sll.sll_family = AF_PACKET;
        reply_sll.sll_ifindex = port.ifindex;
        reply_sll.sll_protocol = htons(ETH_P_IP);

        for (int rounds = 0; rounds < 8; ++rounds) {
            for (int k = 0; k < kRecvBatch; ++k) {
                riovs[k].iov_base = rx_batch_buf[k];
                riovs[k].iov_len = sizeof(rx_batch_buf[k]);
                rmsgs[k].msg_hdr.msg_name = &raddrs[k];
                rmsgs[k].msg_hdr.msg_namelen = sizeof(raddrs[k]);
                rmsgs[k].msg_hdr.msg_iov = &riovs[k];
                rmsgs[k].msg_hdr.msg_iovlen = 1;
            }
            int cnt = recvmmsg(port.raw_sock, rmsgs, kRecvBatch, MSG_DONTWAIT, nullptr);
            if (cnt <= 0) break;

            uint64_t now_us = get_time_us();
            int reply_cnt = 0;

            for (int k = 0; k < cnt; ++k) {
                if (raddrs[k].sll_pkttype == PACKET_OUTGOING) continue;
                ssize_t len = static_cast<ssize_t>(rmsgs[k].msg_len);
                if (len < 62) continue;

                uint8_t* rx_buf = rx_batch_buf[k];
                auto* eth = reinterpret_cast<struct ether_header*>(rx_buf);
                if (ntohs(eth->ether_type) != ETHERTYPE_IP) continue;
                if (std::memcmp(eth->ether_shost, port.mac, 6) == 0) continue;

                auto* ip = reinterpret_cast<struct iphdr*>(rx_buf + 14);
                int hlen = static_cast<int>(ip->ihl) << 2;
                const int min_need = 14 + hlen + static_cast<int>(sizeof(struct icmphdr)) + 11;
                if (hlen < 20 || len < min_need) continue;
                if (ip->protocol != IPPROTO_ICMP) continue;

                auto* icmp = reinterpret_cast<struct icmphdr*>(rx_buf + 14 + hlen);
                uint8_t* payload = rx_buf + 14 + hlen + sizeof(struct icmphdr);
                if (std::memcmp(payload, "RK_NET_PING", 11) != 0) continue;

                port.rx_bytes += static_cast<uint64_t>(len);
                port.rx_bytes_win += static_cast<uint64_t>(len);

                if (icmp->type == ICMP_ECHO) {
                    std::memcpy(eth->ether_dhost, eth->ether_shost, 6);
                    std::memcpy(eth->ether_shost, port.mac, 6);

                    ip->daddr = ip->saddr;
                    ip->saddr = port.local_ip_be;
                    ip->check = 0;
                    ip->check = in_cksum(ip, hlen);

                    icmp->type = ICMP_ECHOREPLY;
                    icmp->checksum = 0;
                    icmp->checksum = high_load.load()
                                         ? in_cksum(icmp, 32)
                                         : in_cksum(icmp, static_cast<int>(len) - 14 - hlen);

                    siovs[reply_cnt].iov_base = rx_buf;
                    siovs[reply_cnt].iov_len = static_cast<size_t>(len);
                    smsgs[reply_cnt].msg_hdr.msg_name = &reply_sll;
                    smsgs[reply_cnt].msg_hdr.msg_namelen = sizeof(reply_sll);
                    smsgs[reply_cnt].msg_hdr.msg_iov = &siovs[reply_cnt];
                    smsgs[reply_cnt].msg_hdr.msg_iovlen = 1;
                    reply_cnt++;
                } else if (icmp->type == ICMP_ECHOREPLY) {
                    if (ntohs(icmp->un.echo.id) == port.ping_id) {
                        uint16_t seq = ntohs(icmp->un.echo.sequence);
                        auto& slot = port.sent_ring[seq & (NetPortInfo::kSentRingSize - 1)];
                        if (slot.active && slot.seq == seq && !slot.replied) {
                            slot.replied = true;
                            slot.active = false;
                            if (port.inflight_cnt > 0) --port.inflight_cnt;
                            port.recv++;
                            double rtt_ms = (now_us >= slot.send_time_us)
                                                ? ((now_us - slot.send_time_us) / 1000.0)
                                                : 0.0;
                            port.rtt_cur = rtt_ms;
                            port.rtt_sum += rtt_ms;
                            if (rtt_ms < port.rtt_min) port.rtt_min = rtt_ms;
                            if (rtt_ms > port.rtt_max) port.rtt_max = rtt_ms;
                        }
                    }
                }
            }

            if (reply_cnt > 0) {
                int r_sent = sendmmsg(port.raw_sock, smsgs, static_cast<unsigned int>(reply_cnt), MSG_DONTWAIT);
                for (int r = 0; r < r_sent; ++r) {
                    port.tx_bytes += siovs[r].iov_len;
                    port.tx_bytes_win += siovs[r].iov_len;
                }
            }

            if (cnt < kRecvBatch) break;
        }
    }
}

void NetEmcTester::tick_timeout_locked() {
    uint64_t now_us = get_time_us();
    const uint64_t timeout_us = 1000000; // 1秒超时

    for (auto& port : net_ports) {
        if (port.inflight_cnt == 0) continue;
        for (auto& slot : port.sent_ring) {
            if (!slot.active) continue;
            if (!slot.replied && !slot.marked_lost && (now_us - slot.send_time_us) > timeout_us) {
                slot.marked_lost = true;
                slot.active = false;
                if (port.inflight_cnt > 0) --port.inflight_cnt;
                port.lost++;
                // 限流日志：每端口每秒最多写 1 条 DROP，防止高吞吐断线时刷爆 /tmp
                if (now_us - port.last_drop_log_us >= 1000000ULL) {
                    port.last_drop_log_us = now_us;
                    log_event_locked("DROP", "WARNING: " + port.ch_name + " Seq #" +
                                     std::to_string(slot.seq) + " packet timeout (ESD/EFT glitch)", 3);
                }
            }
        }
    }
}

void NetEmcTester::thread_loop() {
    auto last_send = std::chrono::steady_clock::now();
    auto last_status = std::chrono::steady_clock::now();
    auto last_timeout = std::chrono::steady_clock::now();

    while (!thread_stop.load()) {
        if (reinit_req.exchange(false)) {
            std::lock_guard<std::mutex> lk(mtx);
            rebind_all_net_sockets_locked();
        }

        auto now = std::chrono::steady_clock::now();

        auto status_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status).count();
        if (status_ms >= 500) {
            double dt_s = status_ms / 1000.0;
            last_status = now;
            update_net_status();
            std::lock_guard<std::mutex> lk(mtx);
            for (auto& p : net_ports) {
                p.tx_mbps = (dt_s > 0.05) ? (static_cast<double>(p.tx_bytes_win) * 8.0 / 1e6 / dt_s) : 0.0;
                p.rx_mbps = (dt_s > 0.05) ? (static_cast<double>(p.rx_bytes_win) * 8.0 / 1e6 / dt_s) : 0.0;
                p.tx_bytes_win = 0;
                p.rx_bytes_win = 0;
            }
        }

        const bool is_hl = high_load.load();

        std::vector<struct pollfd> pfds;
        bool need_work = active.load();
        bool any_burst = false;
        bool burst_need_refill = false;
        {
            std::lock_guard<std::mutex> lk(mtx);
            if (need_work) {
                if (is_hl ||
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() >= interval_ms) {
                    if (!is_hl) last_send = now;
                    tick_send_locked();
                }
            }

            for (const auto& p : net_ports) {
                if (p.inflight_cnt > 0) need_work = true;
                if (p.raw_sock >= 0) {
                    pfds.push_back({p.raw_sock, POLLIN, 0});
                }
                if (is_hl && p.enabled && p.carrier &&
                    p.peer_idx >= 0 && p.peer_idx < static_cast<int>(net_ports.size()) &&
                    net_ports[static_cast<size_t>(p.peer_idx)].enabled &&
                    net_ports[static_cast<size_t>(p.peer_idx)].carrier) {
                    any_burst = true;
                    size_t max_win = (p.speed >= 1000) ? 48 : ((p.speed >= 100) ? 12 : 4);
                    if (p.inflight_cnt < max_win) burst_need_refill = true;
                }
            }

            if (need_work &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_timeout).count() >= 50) {
                last_timeout = now;
                tick_timeout_locked();
            }
        }

        auto after_work = std::chrono::steady_clock::now();
        int status_remain = 500 - static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(after_work - last_status).count());
        if (status_remain < 1) status_remain = 1;

        int wait_ms = std::min(100, status_remain);
        if (active.load()) {
            if (any_burst) {
                // 窗口未满时 1ms 内继续补发；窗口已满时在 poll() 阻塞等待回包即刻自时钟唤醒
                wait_ms = burst_need_refill ? 1 : std::min(50, status_remain);
            } else {
                int elapsed = static_cast<int>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(after_work - last_send).count());
                int remain = std::max(1, interval_ms - elapsed);
                wait_ms = std::min(remain, status_remain);
            }
        } else if (need_work) {
            wait_ms = std::min(50, status_remain);
        }

        if (wake_pipe[0] >= 0) {
            pfds.push_back({wake_pipe[0], POLLIN, 0});
        }
        if (!pfds.empty()) {
            int pr = poll(pfds.data(), pfds.size(), wait_ms);
            if (pr > 0) {
                bool got_sock_in = false;
                for (const auto& pfd : pfds) {
                    if (pfd.fd == wake_pipe[0]) {
                        if (pfd.revents & POLLIN) {
                            uint8_t drain_b[16];
                            while (read(wake_pipe[0], drain_b, sizeof(drain_b)) > 0) {}
                        }
                    } else if (pfd.revents & POLLIN) {
                        got_sock_in = true;
                    }
                }
                if (got_sock_in) {
                    std::lock_guard<std::mutex> lk(mtx);
                    tick_recv_locked();
                } else if (!thread_stop.load()) {
                    usleep(5000);
                }
            }
        } else {
            usleep(static_cast<useconds_t>(wait_ms * 1000));
        }
    }
}
