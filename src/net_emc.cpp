#include "net_emc.hpp"
#include "common.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
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

NetPortInfo::NetPortInfo(std::string n, std::string ch, std::string peer, int pidx, std::string lip, std::string pip)
    : name(std::move(n)), ch_name(std::move(ch)), peer_name(std::move(peer)),
      local_ip(lip), peer_ip(pip), current_ip(lip), peer_idx(pidx) {}

NetEmcTester::NetEmcTester() {
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
    if (worker.joinable()) worker.join();

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
        ps.rtt_cur = p.rtt_cur;
        ps.rtt_min = p.rtt_min;
        ps.rtt_max = p.rtt_max;
        ps.rtt_sum = p.rtt_sum;
        snap.total_sent += p.sent;
        snap.total_recv += p.recv;
        snap.total_lost += p.lost;
        snap.total_flaps += p.flaps;
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

void NetEmcTester::reset_stats() {
    std::lock_guard<std::mutex> lk(mtx);
    for (auto& p : net_ports) {
        p.sent = 0;
        p.recv = 0;
        p.lost = 0;
        p.flaps = 0;
        p.rtt_cur = 0.0;
        p.rtt_min = 999999.0;
        p.rtt_max = 0.0;
        p.rtt_sum = 0.0;
        p.sent_map.clear();
    }
}

void NetEmcTester::log_event(const std::string& level, const std::string& msg, int color) {
    std::lock_guard<std::mutex> lk(mtx);
    log_event_locked(level, msg, color);
}

void NetEmcTester::setup_default_network() {
    if (access("/usr/local/sbin/nm-eth-names.sh", X_OK) == 0 && access("/sys/class/net/nm_i210", F_OK) == 0) {
        if (system("/usr/local/sbin/nm-eth-names.sh >/dev/null 2>&1") < 0) {}
    }

    // 清理旧版同网段遗留 IP，避免与独立子网配置产生主路由冲突
    if (system("ip addr del 192.168.10.2/24 dev eth1 2>/dev/null; "
               "ip addr del 192.168.20.2/24 dev eth3 2>/dev/null") < 0) {}

    struct NetCfg {
        const char* iface;
        const char* ip;
        const char* brd;
    } cfgs[] = {
        {"eth0", "192.168.10.1", "192.168.10.255"},
        {"eth1", "192.168.11.1", "192.168.11.255"},
        {"eth2", "192.168.20.1", "192.168.20.255"},
        {"eth3", "192.168.21.1", "192.168.21.255"}
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
    net_ports.emplace_back("eth0", "Port 1 (eth0->eth1)", "eth1", 1, "192.168.10.1", "192.168.11.1");
    net_ports.emplace_back("eth1", "Port 2 (eth1->eth0)", "eth0", 0, "192.168.11.1", "192.168.10.1");
    net_ports.emplace_back("eth2", "Port 3 (eth2->eth3)", "eth3", 3, "192.168.20.1", "192.168.21.1");
    net_ports.emplace_back("eth3", "Port 4 (eth3->eth2)", "eth2", 2, "192.168.21.1", "192.168.20.1");
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
        std::ifstream fc("/sys/class/net/" + iface + "/carrier");
        int c = 0;
        if (fc.is_open() && (fc >> c)) states[i].carrier = (c == 1);

        std::ifstream fs("/sys/class/net/" + iface + "/speed");
        int sp = -1;
        if (fs.is_open() && (fs >> sp)) states[i].speed = sp;

        states[i].ifindex = static_cast<int>(if_nametoindex(iface.c_str()));
        states[i].has_mac = read_iface_mac(iface, states[i].mac);
    }

    std::lock_guard<std::mutex> lk(mtx);
    bool mac_updated = false;
    for (size_t i = 0; i < net_ports.size() && i < states.size(); ++i) {
        auto& port = net_ports[i];
        const auto& st = states[i];

        port.current_ip = port.local_ip;
        port.speed = st.speed; // 先更新速率，确保 Link Up 日志打印真实协商速率

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

    for (auto& port : net_ports) {
        if (!port.enabled) continue;

        if (!port.carrier || port.raw_sock < 0 || port.ifindex <= 0) {
            port.sent++;
            port.lost++;
            continue;
        }

        int pkt_len = std::max(64, packet_size);
        std::vector<uint8_t> pkt(pkt_len, 0);

        // 1. Ethernet Header (优先使用对端真实单播 MAC，避免广播泛洪)
        auto* eth = reinterpret_cast<struct ether_header*>(pkt.data());
        if (port.has_peer_mac) {
            std::memcpy(eth->ether_dhost, port.peer_mac, 6);
        } else {
            std::memset(eth->ether_dhost, 0xff, 6);
        }
        std::memcpy(eth->ether_shost, port.mac, 6);
        eth->ether_type = htons(ETHERTYPE_IP);

        uint64_t now_us = get_time_us();
        uint16_t seq = static_cast<uint16_t>(port.sent & 0xFFFFu);

        // 2. IPv4 Header
        auto* ip = reinterpret_cast<struct iphdr*>(pkt.data() + 14);
        ip->version = 4;
        ip->ihl = 5;
        ip->tot_len = htons(static_cast<uint16_t>(pkt_len - 14));
        ip->id = htons(seq);
        ip->ttl = 64;
        ip->protocol = IPPROTO_ICMP;
        ip->saddr = inet_addr(port.local_ip.c_str());
        ip->daddr = inet_addr(port.peer_ip.c_str());
        ip->check = in_cksum(ip, 20);

        // 3. ICMP Echo Request Header
        auto* icmp = reinterpret_cast<struct icmphdr*>(pkt.data() + 34);
        icmp->type = ICMP_ECHO;
        icmp->code = 0;
        icmp->un.echo.id = htons(port.ping_id);
        icmp->un.echo.sequence = htons(seq);

        // Payload: Magic string + timestamp
        std::memcpy(pkt.data() + 42, "RK_NET_PING", 11);
        std::memcpy(pkt.data() + 54, &now_us, sizeof(now_us));
        icmp->checksum = in_cksum(icmp, pkt_len - 34);

        struct sockaddr_ll sll{};
        sll.sll_family = AF_PACKET;
        sll.sll_ifindex = port.ifindex;
        sll.sll_protocol = htons(ETH_P_IP);

        ssize_t ret = sendto(port.raw_sock, pkt.data(), pkt_len, 0,
                             reinterpret_cast<struct sockaddr*>(&sll), sizeof(sll));
        if (ret > 0) {
            port.sent++;
            SentPacket sp;
            sp.send_time_us = now_us;
            sp.replied = false;
            sp.marked_lost = false;
            port.sent_map[seq] = sp;
        } else {
            port.sent++;
            port.lost++;
        }
    }
}

void NetEmcTester::tick_recv_locked() {
    uint8_t rx_buf[2048];
    for (auto& port : net_ports) {
        if (port.raw_sock < 0 || port.ifindex <= 0) continue;

        while (true) {
            struct sockaddr_ll from_sll{};
            socklen_t fromlen = sizeof(from_sll);
            ssize_t len = recvfrom(port.raw_sock, rx_buf, sizeof(rx_buf), 0,
                                   reinterpret_cast<struct sockaddr*>(&from_sll), &fromlen);
            if (len <= 0) break;
            if (from_sll.sll_pkttype == PACKET_OUTGOING) continue;
            if (len < 62) continue;

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

            uint64_t now_us = get_time_us();

            if (icmp->type == ICMP_ECHO) {
                // 收到对端 Echo Request -> 通过物理网线原路返回 Echo Reply
                std::memcpy(eth->ether_dhost, eth->ether_shost, 6);
                std::memcpy(eth->ether_shost, port.mac, 6);

                ip->daddr = ip->saddr;
                ip->saddr = inet_addr(port.local_ip.c_str());
                ip->check = 0;
                ip->check = in_cksum(ip, hlen);

                icmp->type = ICMP_ECHOREPLY;
                icmp->checksum = 0;
                icmp->checksum = in_cksum(icmp, static_cast<int>(len) - 14 - hlen);

                struct sockaddr_ll sll{};
                sll.sll_family = AF_PACKET;
                sll.sll_ifindex = port.ifindex;
                sll.sll_protocol = htons(ETH_P_IP);

                if (sendto(port.raw_sock, rx_buf, static_cast<size_t>(len), 0,
                           reinterpret_cast<struct sockaddr*>(&sll), sizeof(sll)) < 0) {}
            } else if (icmp->type == ICMP_ECHOREPLY) {
                if (ntohs(icmp->un.echo.id) == port.ping_id) {
                    uint16_t seq = ntohs(icmp->un.echo.sequence);
                    auto it = port.sent_map.find(seq);
                    if (it != port.sent_map.end() && !it->second.replied) {
                        it->second.replied = true;
                        port.recv++;
                        double rtt_ms = (now_us - it->second.send_time_us) / 1000.0;
                        port.rtt_cur = rtt_ms;
                        port.rtt_sum += rtt_ms;
                        if (rtt_ms < port.rtt_min) port.rtt_min = rtt_ms;
                        if (rtt_ms > port.rtt_max) port.rtt_max = rtt_ms;
                    }
                }
            }
        }
    }
}

void NetEmcTester::tick_timeout_locked() {
    uint64_t now_us = get_time_us();
    const uint64_t timeout_us = 1000000; // 1秒超时

    for (auto& port : net_ports) {
        auto it = port.sent_map.begin();
        while (it != port.sent_map.end()) {
            if (!it->second.replied && !it->second.marked_lost) {
                if (now_us - it->second.send_time_us > timeout_us) {
                    it->second.marked_lost = true;
                    port.lost++;
                    log_event_locked("DROP", "WARNING: " + port.ch_name + " Seq #" +
                                     std::to_string(it->first) + " packet timeout (ESD/EFT glitch)", 3);
                }
            }
            if (now_us - it->second.send_time_us > 4000000) {
                it = port.sent_map.erase(it);
            } else {
                ++it;
            }
        }
    }
}

void NetEmcTester::thread_loop() {
    auto last_send = std::chrono::steady_clock::now();
    auto last_status = std::chrono::steady_clock::now();

    while (!thread_stop.load()) {
        if (reinit_req.exchange(false)) {
            std::lock_guard<std::mutex> lk(mtx);
            rebind_all_net_sockets_locked();
        }

        auto now = std::chrono::steady_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status).count() >= 500) {
            last_status = now;
            update_net_status();
        }

        std::vector<struct pollfd> pfds;
        bool need_work = active.load();
        {
            std::lock_guard<std::mutex> lk(mtx);
            if (need_work &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() >= interval_ms) {
                last_send = now;
                tick_send_locked();
            }

            for (const auto& p : net_ports) {
                if (!p.sent_map.empty()) need_work = true;
                if (p.raw_sock >= 0) {
                    pfds.push_back({p.raw_sock, POLLIN, 0});
                }
            }

            if (need_work) {
                tick_timeout_locked();
            }
        }

        int wait_ms = 20;
        if (active.load()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - last_send)
                               .count();
            int remain = static_cast<int>(interval_ms - elapsed);
            wait_ms = std::max(1, std::min(10, remain));
        } else if (!need_work) {
            wait_ms = 50;
        }

        if (!pfds.empty()) {
            int pr = poll(pfds.data(), pfds.size(), wait_ms);
            if (pr > 0) {
                bool got_in = false;
                for (const auto& pfd : pfds) {
                    if (pfd.revents & POLLIN) got_in = true;
                }
                if (got_in) {
                    std::lock_guard<std::mutex> lk(mtx);
                    tick_recv_locked();
                } else {
                    usleep(5000);
                }
            }
        } else {
            usleep(static_cast<useconds_t>(wait_ms * 1000));
        }
    }
}
