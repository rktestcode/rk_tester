#include "can_tester.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

CanBusTester::~CanBusTester() {
    stop();
}

void CanBusTester::cycle_direction() {
    int d = (static_cast<int>(dir.load()) + 1) % 3;
    dir.store(static_cast<CanDirection>(d));
    if (!active.load()) reset();
}

void CanBusTester::set_direction(CanDirection d) {
    dir.store(d);
    if (!active.load()) reset();
}

void CanBusTester::toggle_fd_mode() {
    fd_mode.store(!fd_mode.load());
    if (!active.load()) reset();
}

void CanBusTester::set_fd_mode(bool en) {
    fd_mode.store(en);
    if (!active.load()) reset();
}

std::string CanBusTester::get_short_label() const {
    if (fd_mode.load()) {
        switch (dir.load()) {
            case CanDirection::CAN0_TO_CAN1: return "CANFD(0->1):";
            case CanDirection::CAN1_TO_CAN0: return "CANFD(1->0):";
            case CanDirection::PING_PONG:    return "CANFD(0<>1):";
        }
        return "CANFD(0->1):";
    }
    switch (dir.load()) {
        case CanDirection::CAN0_TO_CAN1: return "CAN(c0->c1):";
        case CanDirection::CAN1_TO_CAN0: return "CAN(c1->c0):";
        case CanDirection::PING_PONG:    return "CAN(c0<>c1):";
    }
    return "CAN(c0->c1):";
}

std::string CanBusTester::get_wide_label() const {
    if (fd_mode.load()) {
        switch (dir.load()) {
            case CanDirection::CAN0_TO_CAN1: return "CANFD(c0->1):";
            case CanDirection::CAN1_TO_CAN0: return "CANFD(c1->0):";
            case CanDirection::PING_PONG:    return "CANFD(0<->1):";
        }
        return "CANFD(c0->1):";
    }
    switch (dir.load()) {
        case CanDirection::CAN0_TO_CAN1: return "CAN(c0->c1):";
        case CanDirection::CAN1_TO_CAN0: return "CAN(c1->c0):";
        case CanDirection::PING_PONG:    return "CAN(c0<->c1):";
    }
    return "CAN(c0->c1):";
}

std::string CanBusTester::get_mode_name() const {
    switch (dir.load()) {
        case CanDirection::CAN0_TO_CAN1: return "CAN0 -> CAN1 (Uni)";
        case CanDirection::CAN1_TO_CAN0: return "CAN1 -> CAN0 (Uni)";
        case CanDirection::PING_PONG:    return "CAN0 <-> CAN1 (Ping-Pong)";
    }
    return "CAN0 -> CAN1 (Uni)";
}

std::string CanBusTester::get_short_mode_name() const {
    switch (dir.load()) {
        case CanDirection::CAN0_TO_CAN1: return "0->1";
        case CanDirection::CAN1_TO_CAN0: return "1->0";
        case CanDirection::PING_PONG:    return "0<->1 PP";
    }
    return "0->1";
}

std::string CanBusTester::get_menu_desc() const {
    const char* spd = fd_mode.load() ? "FD64B 2M" : "500k";
    switch (dir.load()) {
        case CanDirection::CAN0_TO_CAN1: return std::string("CAN0->CAN1 ") + spd + " [D/F]";
        case CanDirection::CAN1_TO_CAN0: return std::string("CAN1->CAN0 ") + spd + " [D/F]";
        case CanDirection::PING_PONG:    return std::string("CAN0<->CAN1 ") + spd + " PP";
    }
    return std::string("CAN0->CAN1 ") + spd + " [D/F]";
}

std::string CanBusTester::get_error_detail() const {
    std::lock_guard<std::mutex> lk(meta_mtx);
    return last_error_detail;
}

std::string CanBusTester::get_bitrate_info() const {
    std::lock_guard<std::mutex> lk(meta_mtx);
    return bitrate_info;
}

void CanBusTester::start() {
    if (active.load()) return;
    if (worker.joinable()) worker.join();
    reset_requested.store(false);
    set_error_detail("");
    state.store(ChannelState::RUNNING);
    active.store(true);
    worker = std::thread(&CanBusTester::run, this);
}

void CanBusTester::stop() {
    active.store(false);
    if (worker.joinable()) worker.join();
    tx_fps.store(0.0);
    rx_fps.store(0.0);
    if (state.load() != ChannelState::PORT_ERROR) {
        state.store(ChannelState::STANDBY);
    }
}

void CanBusTester::reset() {
    if (active.load()) {
        reset_requested.store(true);
    } else {
        clear_counters();
        set_error_detail("");
        if (state.load() == ChannelState::PORT_ERROR) {
            state.store(ChannelState::STANDBY);
        }
    }
}

void CanBusTester::set_error_detail(const std::string& msg) {
    std::lock_guard<std::mutex> lk(meta_mtx);
    last_error_detail = msg;
}

void CanBusTester::set_bitrate_info(const std::string& info) {
    std::lock_guard<std::mutex> lk(meta_mtx);
    bitrate_info = info;
}

void CanBusTester::clear_counters() {
    tx_bytes.store(0);
    rx_bytes.store(0);
    tx_frames.store(0);
    rx_frames.store(0);
    err_frames.store(0);
    crc_errors.store(0);
    bus_errors.store(0);
    tx_fps.store(0.0);
    rx_fps.store(0.0);
    rtt_cur_ms.store(0.0);
    rtt_max_ms.store(0.0);
    rtt_sum_ms.store(0.0);
    last_tx_seq.store(0);
    last_rx_seq.store(0);
    last_valid_rx_us.store(0);
}

bool CanBusTester::is_iface_up(const std::string& dev) {
    std::ifstream f("/sys/class/net/" + dev + "/flags");
    if (!f.is_open()) return false;
    unsigned int flags = 0;
    f >> std::hex >> flags;
    return (flags & IFF_UP) != 0;
}

int CanBusTester::get_iface_mtu(const std::string& dev) {
    std::ifstream f("/sys/class/net/" + dev + "/mtu");
    int mtu = 0;
    if (f.is_open()) f >> mtu;
    return mtu;
}

std::string CanBusTester::ensure_can_up(const std::string& dev, bool force_fd_reinit) {
    std::string mode = "500k";
    bool is_up = is_iface_up(dev);
    int mtu = get_iface_mtu(dev);
    if (!is_up || (force_fd_reinit && mtu < static_cast<int>(CANFD_MTU))) {
        if (is_up) {
            std::string down_cmd = "ip link set " + dev + " down 2>/dev/null";
            if (system(down_cmd.c_str()) < 0) {}
        }
        std::string fd_cmd = "ip link set " + dev + " type can bitrate 500000 dbitrate 2000000 fd on 2>/dev/null";
        if (system(fd_cmd.c_str()) == 0) {
            mode = "500k/2M FD";
        } else {
            std::string std_cmd = "ip link set " + dev + " type can bitrate 500000 2>/dev/null";
            if (system(std_cmd.c_str()) == 0) mode = "500k 2.0";
        }
        std::string up_cmd = "ip link set " + dev + " txqueuelen 1000 2>/dev/null; "
                             "ip link set " + dev + " up 2>/dev/null";
        if (system(up_cmd.c_str()) < 0) {}
    } else {
        std::string cmd = "ip link set " + dev + " txqueuelen 1000 2>/dev/null";
        if (system(cmd.c_str()) < 0) {}
        mode = (mtu >= static_cast<int>(CANFD_MTU)) ? "500k/2M FD" : "500k UP";
    }
    return mode;
}

void CanBusTester::flush_can_socket(int s) {
    struct canfd_frame stale{};
    while (read(s, &stale, sizeof(stale)) > 0) {}
}

int CanBusTester::open_socket(const std::string& ifname, canid_t filter_id, int& out_errno) {
    out_errno = 0;
    int s = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, CAN_RAW);
    if (s < 0) {
        out_errno = errno;
        return -1;
    }
    int enable_fd = 1;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd));

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        out_errno = errno;
        close(s);
        return -1;
    }
    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        out_errno = errno;
        close(s);
        return -1;
    }

    struct can_filter rfilter[1];
    rfilter[0].can_id = filter_id;
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    // 启用总线硬件错误帧监控（Bus-Off / ACK 缺失 / 总线位错误）
    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_CRTL | CAN_ERR_ACK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    flush_can_socket(s);
    return s;
}

void CanBusTester::process_rx(int s0, int s1, CanDirection cur_dir, bool cur_fd,
                              std::deque<PendingCanFrame>& pending_tx,
                              uint64_t& rx_calc) {
    uint64_t now_us = get_time_us();
    const uint8_t exp_len = cur_fd ? 64 : 8;
    const ssize_t exp_mtu = cur_fd ? static_cast<ssize_t>(CANFD_MTU) : static_cast<ssize_t>(CAN_MTU);

    auto match_rx = [&](uint8_t seq, const uint8_t* rx_data, uint8_t len) {
        auto it = std::find_if(pending_tx.begin(), pending_tx.end(),
                               [&](const auto& item) { return item.seq == seq && item.len == len; });
        if (it != pending_tx.end()) {
            // 逐字节比对全部载荷（8B 或 64B）是否与预期发送数据完全一致
            if (std::memcmp(rx_data, it->expect_data, len) == 0) {
                double rtt_ms = (now_us >= it->send_time_us) ? ((now_us - it->send_time_us) / 1000.0) : 0.0;
                pending_tx.erase(it);
                rx_frames.fetch_add(1);
                rx_bytes.fetch_add(len);
                last_rx_seq.store(seq);
                last_valid_rx_us.store(now_us);
                rtt_cur_ms.store(rtt_ms);
                if (rtt_ms > rtt_max_ms.load()) rtt_max_ms.store(rtt_ms);
                rtt_sum_ms.store(rtt_sum_ms.load() + rtt_ms);
                rx_calc++;
            } else {
                pending_tx.erase(it);
                crc_errors.fetch_add(1);
                err_frames.fetch_add(1);
            }
        }
    };

    auto verify_sum = [](const uint8_t* data, uint8_t len) -> bool {
        uint8_t sum = 0;
        for (uint8_t i = 0; i + 1 < len; ++i) sum += data[i];
        return sum == data[len - 1];
    };

    if (cur_dir == CanDirection::CAN0_TO_CAN1) {
        while (true) {
            struct canfd_frame rf1{};
            ssize_t r = read(s1, &rf1, sizeof(rf1));
            if (r < static_cast<ssize_t>(CAN_MTU)) break;
            if (rf1.can_id & CAN_ERR_FLAG) {
                bus_errors.fetch_add(1);
                continue;
            }
            if (r == exp_mtu && rf1.can_id == 0x121 && rf1.len == exp_len && rf1.data[0] == 0xC0) {
                if (verify_sum(rf1.data, exp_len)) match_rx(rf1.data[1], rf1.data, exp_len);
                else crc_errors.fetch_add(1);
            }
        }
        // 也统计发送端 s0 的总线错误帧（如无 ACK / Bus-Off）
        while (true) {
            struct canfd_frame ef0{};
            ssize_t r = read(s0, &ef0, sizeof(ef0));
            if (r < static_cast<ssize_t>(CAN_MTU)) break;
            if (ef0.can_id & CAN_ERR_FLAG) bus_errors.fetch_add(1);
        }
    } else if (cur_dir == CanDirection::CAN1_TO_CAN0) {
        while (true) {
            struct canfd_frame rf0{};
            ssize_t r = read(s0, &rf0, sizeof(rf0));
            if (r < static_cast<ssize_t>(CAN_MTU)) break;
            if (rf0.can_id & CAN_ERR_FLAG) {
                bus_errors.fetch_add(1);
                continue;
            }
            if (r == exp_mtu && rf0.can_id == 0x221 && rf0.len == exp_len && rf0.data[0] == 0xC1) {
                if (verify_sum(rf0.data, exp_len)) match_rx(rf0.data[1], rf0.data, exp_len);
                else crc_errors.fetch_add(1);
            }
        }
        while (true) {
            struct canfd_frame ef1{};
            ssize_t r = read(s1, &ef1, sizeof(ef1));
            if (r < static_cast<ssize_t>(CAN_MTU)) break;
            if (ef1.can_id & CAN_ERR_FLAG) bus_errors.fetch_add(1);
        }
    } else if (cur_dir == CanDirection::PING_PONG) {
        while (true) {
            struct canfd_frame rf1{};
            ssize_t r = read(s1, &rf1, sizeof(rf1));
            if (r < static_cast<ssize_t>(CAN_MTU)) break;
            if (rf1.can_id & CAN_ERR_FLAG) {
                bus_errors.fetch_add(1);
                continue;
            }
            if (r == exp_mtu && rf1.can_id == 0x121 && rf1.len == exp_len && rf1.data[0] == 0xC0) {
                bool payload_ok = verify_sum(rf1.data, exp_len);
                for (uint8_t i = 2; i + 1 < exp_len && payload_ok; ++i) {
                    if (rf1.data[i] != static_cast<uint8_t>((rf1.data[1] + i * 3) & 0xFF)) {
                        payload_ok = false;
                    }
                }
                if (payload_ok) {
                    struct canfd_frame pong{};
                    pong.can_id = 0x221;
                    pong.len = exp_len;
                    pong.flags = cur_fd ? CANFD_BRS : 0;
                    pong.data[0] = 0xC1;
                    pong.data[1] = rf1.data[1];
                    for (uint8_t i = 2; i + 1 < exp_len; ++i) {
                        pong.data[i] = static_cast<uint8_t>(rf1.data[i] ^ 0x5Au);
                    }
                    uint8_t psum = 0;
                    for (uint8_t i = 0; i + 1 < exp_len; ++i) psum += pong.data[i];
                    pong.data[exp_len - 1] = psum;
                    if (write(s1, &pong, static_cast<size_t>(exp_mtu)) < 0) {}
                } else {
                    crc_errors.fetch_add(1);
                }
            }
        }

        while (true) {
            struct canfd_frame rf0{};
            ssize_t r = read(s0, &rf0, sizeof(rf0));
            if (r < static_cast<ssize_t>(CAN_MTU)) break;
            if (rf0.can_id & CAN_ERR_FLAG) {
                bus_errors.fetch_add(1);
                continue;
            }
            if (r == exp_mtu && rf0.can_id == 0x221 && rf0.len == exp_len && rf0.data[0] == 0xC1) {
                if (verify_sum(rf0.data, exp_len)) match_rx(rf0.data[1], rf0.data, exp_len);
                else crc_errors.fetch_add(1);
            }
        }
    }
}

void CanBusTester::run() {
    bool req_fd_init = fd_mode.load();
    std::string m0 = ensure_can_up(if0, req_fd_init);
    ensure_can_up(if1, req_fd_init);
    bool hw_fd_ok = (get_iface_mtu(if0) >= static_cast<int>(CANFD_MTU) &&
                     get_iface_mtu(if1) >= static_cast<int>(CANFD_MTU));
    set_bitrate_info(hw_fd_ok ? "500k/2M FD" : m0);

    int err0 = 0, err1 = 0;
    int s0 = open_socket(if0, 0x221, err0); // can0 listens for 0x221
    int s1 = open_socket(if1, 0x121, err1); // can1 listens for 0x121

    if (s0 < 0 || s1 < 0) {
        std::string detail = (s0 < 0) ? (if0 + " (" + std::strerror(err0) + ")")
                                      : (if1 + " (" + std::strerror(err1) + ")");
        set_error_detail(detail);
        state.store(ChannelState::PORT_ERROR);
        if (s0 >= 0) close(s0);
        if (s1 >= 0) close(s1);
        active.store(false);
        return;
    }

    state.store(ChannelState::RUNNING);

    // 提升为低层级实时优先级（SCHED_FIFO 20 + SCHED_RESET_ON_FORK，低于内核 IRQ FIFO 50），
    // 消除满载 CPU 竞争引起的 CAN 用户态唤醒抖动与 RTT 尖峰（置于 ensure_can_up 之后避免影响子命令）
    struct sched_param rt_sp{};
    rt_sp.sched_priority = 20;
    int rt_pol = SCHED_FIFO;
#ifdef SCHED_RESET_ON_FORK
    rt_pol |= SCHED_RESET_ON_FORK;
#endif
    if (pthread_setschedparam(pthread_self(), rt_pol, &rt_sp) != 0) {
        setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), -10);
    }

    uint8_t tx_seq = 0;
    std::deque<PendingCanFrame> pending_tx;

    auto next_tx = std::chrono::steady_clock::now();
    auto last_calc = std::chrono::steady_clock::now();
    auto settle_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(15);
    bool in_settle = true;
    bool rx_ready = true;
    uint64_t tx_calc = 0;
    uint64_t rx_calc = 0;

    CanDirection cur_dir = dir.load();
    bool cur_fd_req = req_fd_init;
    bool cur_use_fd = cur_fd_req && hw_fd_ok;

    while (active.load()) {
        CanDirection req_dir = dir.load();
        bool req_fd = fd_mode.load();
        bool do_reset = reset_requested.exchange(false);
        if (req_dir != cur_dir || req_fd != cur_fd_req || do_reset) {
            bool mode_changed = (req_dir != cur_dir || req_fd != cur_fd_req);
            if (req_fd && !hw_fd_ok) {
                ensure_can_up(if0, true);
                ensure_can_up(if1, true);
                hw_fd_ok = (get_iface_mtu(if0) >= static_cast<int>(CANFD_MTU) &&
                            get_iface_mtu(if1) >= static_cast<int>(CANFD_MTU));
                set_bitrate_info(hw_fd_ok ? "500k/2M FD" : "500k 2.0");
            }
            cur_dir = req_dir;
            cur_fd_req = req_fd;
            cur_use_fd = cur_fd_req && hw_fd_ok;
            flush_can_socket(s0);
            flush_can_socket(s1);
            pending_tx.clear();
            tx_seq = 0;
            tx_calc = 0;
            rx_calc = 0;
            clear_counters();
            auto tnow = std::chrono::steady_clock::now();
            last_calc = tnow;
            settle_until = tnow + std::chrono::milliseconds(mode_changed ? 30 : 15);
            in_settle = true;
            rx_ready = true;
        }

        auto now = std::chrono::steady_clock::now();
        if (in_settle) {
            flush_can_socket(s0);
            flush_can_socket(s1);
            if (now < settle_until) {
                usleep(2000);
                continue;
            }
            flush_can_socket(s0);
            flush_can_socket(s1);
            in_settle = false;
            next_tx = now;
            last_calc = now;
            rx_ready = true;
        }

        uint64_t now_us = get_time_us();
        const int tx_interval_ms = cur_fd_req ? 2 : 20; // CAN-FD 高频压力档 2ms (500fps)，标准档 20ms (50fps)
        const auto tx_period = std::chrono::milliseconds(tx_interval_ms);
        const uint64_t rx_timeout_us = cur_fd_req ? 200000 : 500000;
        const uint8_t frame_len = cur_use_fd ? 64 : 8;
        const ssize_t frame_mtu = cur_use_fd ? static_cast<ssize_t>(CANFD_MTU) : static_cast<ssize_t>(CAN_MTU);

        // 1. TX（固定相位递增，彻底消除 20ms -> 21.2ms 周期漂移）
        if (now >= next_tx) {
            next_tx += tx_period;
            if (next_tx <= now - tx_period) {
                next_tx = now + tx_period;
            }

            if (cur_dir == CanDirection::CAN0_TO_CAN1 || cur_dir == CanDirection::PING_PONG) {
                struct canfd_frame f0{};
                f0.can_id = 0x121;
                f0.len = frame_len;
                f0.flags = cur_use_fd ? CANFD_BRS : 0;
                f0.data[0] = 0xC0;
                f0.data[1] = tx_seq;
                for (uint8_t i = 2; i + 1 < frame_len; ++i) {
                    f0.data[i] = static_cast<uint8_t>((tx_seq + i * 3) & 0xFF);
                }
                uint8_t sum = 0;
                for (uint8_t i = 0; i + 1 < frame_len; ++i) sum += f0.data[i];
                f0.data[frame_len - 1] = sum;

                ssize_t w = write(s0, &f0, static_cast<size_t>(frame_mtu));
                if (w == frame_mtu) {
                    tx_frames.fetch_add(1);
                    tx_bytes.fetch_add(frame_len);
                    last_tx_seq.store(tx_seq);
                    tx_calc++;
                    PendingCanFrame pf{};
                    pf.seq = tx_seq;
                    pf.len = frame_len;
                    pf.send_time_us = now_us;
                    if (cur_dir == CanDirection::PING_PONG) {
                        pf.expect_data[0] = 0xC1;
                        pf.expect_data[1] = tx_seq;
                        for (uint8_t i = 2; i + 1 < frame_len; ++i) {
                            pf.expect_data[i] = static_cast<uint8_t>(f0.data[i] ^ 0x5Au);
                        }
                        uint8_t psum = 0;
                        for (uint8_t i = 0; i + 1 < frame_len; ++i) psum += pf.expect_data[i];
                        pf.expect_data[frame_len - 1] = psum;
                    } else {
                        std::memcpy(pf.expect_data, f0.data, frame_len);
                    }
                    pending_tx.push_back(pf);
                    tx_seq++;
                } else {
                    tx_frames.fetch_add(1);
                    err_frames.fetch_add(1);
                }
            } else if (cur_dir == CanDirection::CAN1_TO_CAN0) {
                struct canfd_frame f1{};
                f1.can_id = 0x221;
                f1.len = frame_len;
                f1.flags = cur_use_fd ? CANFD_BRS : 0;
                f1.data[0] = 0xC1;
                f1.data[1] = tx_seq;
                for (uint8_t i = 2; i + 1 < frame_len; ++i) {
                    f1.data[i] = static_cast<uint8_t>((tx_seq + i * 5) & 0xFF);
                }
                uint8_t sum = 0;
                for (uint8_t i = 0; i + 1 < frame_len; ++i) sum += f1.data[i];
                f1.data[frame_len - 1] = sum;

                ssize_t w = write(s1, &f1, static_cast<size_t>(frame_mtu));
                if (w == frame_mtu) {
                    tx_frames.fetch_add(1);
                    tx_bytes.fetch_add(frame_len);
                    last_tx_seq.store(tx_seq);
                    tx_calc++;
                    PendingCanFrame pf{};
                    pf.seq = tx_seq;
                    pf.len = frame_len;
                    pf.send_time_us = now_us;
                    std::memcpy(pf.expect_data, f1.data, frame_len);
                    pending_tx.push_back(pf);
                    tx_seq++;
                } else {
                    tx_frames.fetch_add(1);
                    err_frames.fetch_add(1);
                }
            }
        }

        // 2. RX（仅在 poll 提示可读时执行系统调用）
        if (rx_ready) {
            process_rx(s0, s1, cur_dir, cur_use_fd, pending_tx, rx_calc);
            rx_ready = false;
        }

        // 3. 超时检测
        now_us = get_time_us();
        while (!pending_tx.empty() && (now_us - pending_tx.front().send_time_us) > rx_timeout_us) {
            pending_tx.pop_front();
            err_frames.fetch_add(1);
        }

        // 4. 每 500ms 刷新 FPS
        auto calc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calc).count();
        if (calc_ms >= 500) {
            double dt = calc_ms / 1000.0;
            last_calc = now;
            if (dt <= 0.0) dt = 0.5;
            tx_fps.store(tx_calc / dt);
            rx_fps.store(rx_calc / dt);
            tx_calc = 0;
            rx_calc = 0;
        }

        // 纳秒级 ppoll() 精确休眠至下一固定发包相位，收到 CAN 帧立即唤醒
        auto after = std::chrono::steady_clock::now();
        long long rem_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(next_tx - after).count();
        if (rem_ns < 0) rem_ns = 0;
        if (rem_ns > 50000000LL) rem_ns = 50000000LL;
        struct timespec ts{};
        ts.tv_sec = static_cast<time_t>(rem_ns / 1000000000LL);
        ts.tv_nsec = static_cast<long>(rem_ns % 1000000000LL);

        struct pollfd pfds[2];
        pfds[0] = {s0, POLLIN, 0};
        pfds[1] = {s1, POLLIN, 0};
        int pr = ppoll(pfds, 2, &ts, nullptr);
        if (pr > 0) {
            if ((pfds[0].revents & POLLIN) || (pfds[1].revents & POLLIN)) {
                rx_ready = true;
            } else {
                usleep(2000);
            }
        }
    }

    // 停止前短暂排空在途帧（等待最多 30ms）
    auto drain_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(30);
    while (!pending_tx.empty() && std::chrono::steady_clock::now() < drain_end) {
        process_rx(s0, s1, cur_dir, cur_use_fd, pending_tx, rx_calc);
        if (!pending_tx.empty()) usleep(1000);
    }
    if (!pending_tx.empty()) {
        err_frames.fetch_add(pending_tx.size());
        pending_tx.clear();
    }

    close(s0);
    close(s1);
    tx_fps.store(0.0);
    rx_fps.store(0.0);
    state.store(ChannelState::STANDBY);
}
