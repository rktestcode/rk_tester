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
#include <sys/ioctl.h>
#include <sys/socket.h>
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

std::string CanBusTester::get_short_label() const {
    switch (dir.load()) {
        case CanDirection::CAN0_TO_CAN1: return "CAN(c0->c1):";
        case CanDirection::CAN1_TO_CAN0: return "CAN(c1->c0):";
        case CanDirection::PING_PONG:    return "CAN(c0<>c1):";
    }
    return "CAN(c0->c1):";
}

std::string CanBusTester::get_wide_label() const {
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
    switch (dir.load()) {
        case CanDirection::CAN0_TO_CAN1: return "CAN0->CAN1 500k Uni [D]";
        case CanDirection::CAN1_TO_CAN0: return "CAN1->CAN0 500k Uni [D]";
        case CanDirection::PING_PONG:    return "CAN0<->CAN1 500k PP [D]";
    }
    return "CAN0->CAN1 500k Uni [D]";
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

std::string CanBusTester::ensure_can_up(const std::string& dev) {
    std::string mode = "500k";
    if (!is_iface_up(dev)) {
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
        mode = "500k UP";
    }
    return mode;
}

void CanBusTester::flush_can_socket(int s) {
    struct can_frame stale{};
    while (read(s, &stale, sizeof(stale)) > 0) {}
}

int CanBusTester::open_socket(const std::string& ifname, canid_t filter_id, int& out_errno) {
    out_errno = 0;
    int s = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, CAN_RAW);
    if (s < 0) {
        out_errno = errno;
        return -1;
    }
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

void CanBusTester::process_rx(int s0, int s1, CanDirection cur_dir,
                              std::deque<PendingCanFrame>& pending_tx,
                              uint64_t& rx_calc) {
    uint64_t now_us = get_time_us();
    auto match_rx = [&](uint8_t seq, const uint8_t rx_data[8]) {
        auto it = std::find_if(pending_tx.begin(), pending_tx.end(),
                               [&](const auto& item) { return item.seq == seq; });
        if (it != pending_tx.end()) {
            // 逐字节比对全部 8 字节 CAN 数据是否与预期发送载荷完全一致
            if (std::memcmp(rx_data, it->expect_data, 8) == 0) {
                double rtt_ms = (now_us >= it->send_time_us) ? ((now_us - it->send_time_us) / 1000.0) : 0.0;
                pending_tx.erase(it);
                rx_frames.fetch_add(1);
                rx_bytes.fetch_add(8);
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

    if (cur_dir == CanDirection::CAN0_TO_CAN1) {
        while (true) {
            struct can_frame rf1{};
            ssize_t r = read(s1, &rf1, sizeof(rf1));
            if (r < static_cast<ssize_t>(sizeof(rf1))) break;
            if (rf1.can_id & CAN_ERR_FLAG) {
                bus_errors.fetch_add(1);
                continue;
            }
            if (rf1.can_id == 0x121 && rf1.can_dlc == 8 && rf1.data[0] == 0xC0) {
                uint8_t sum = 0;
                for (int i = 0; i < 7; ++i) sum += rf1.data[i];
                if (sum == rf1.data[7]) match_rx(rf1.data[1], rf1.data);
                else crc_errors.fetch_add(1);
            }
        }
        // 也统计发送端 s0 的总线错误帧（如无 ACK / Bus-Off）
        while (true) {
            struct can_frame ef0{};
            ssize_t r = read(s0, &ef0, sizeof(ef0));
            if (r < static_cast<ssize_t>(sizeof(ef0))) break;
            if (ef0.can_id & CAN_ERR_FLAG) bus_errors.fetch_add(1);
        }
    } else if (cur_dir == CanDirection::CAN1_TO_CAN0) {
        while (true) {
            struct can_frame rf0{};
            ssize_t r = read(s0, &rf0, sizeof(rf0));
            if (r < static_cast<ssize_t>(sizeof(rf0))) break;
            if (rf0.can_id & CAN_ERR_FLAG) {
                bus_errors.fetch_add(1);
                continue;
            }
            if (rf0.can_id == 0x221 && rf0.can_dlc == 8 && rf0.data[0] == 0xC1) {
                uint8_t sum = 0;
                for (int i = 0; i < 7; ++i) sum += rf0.data[i];
                if (sum == rf0.data[7]) match_rx(rf0.data[1], rf0.data);
                else crc_errors.fetch_add(1);
            }
        }
        while (true) {
            struct can_frame ef1{};
            ssize_t r = read(s1, &ef1, sizeof(ef1));
            if (r < static_cast<ssize_t>(sizeof(ef1))) break;
            if (ef1.can_id & CAN_ERR_FLAG) bus_errors.fetch_add(1);
        }
    } else if (cur_dir == CanDirection::PING_PONG) {
        while (true) {
            struct can_frame rf1{};
            ssize_t r = read(s1, &rf1, sizeof(rf1));
            if (r < static_cast<ssize_t>(sizeof(rf1))) break;
            if (rf1.can_id & CAN_ERR_FLAG) {
                bus_errors.fetch_add(1);
                continue;
            }
            if (rf1.can_id == 0x121 && rf1.can_dlc == 8 && rf1.data[0] == 0xC0) {
                uint8_t sum = 0;
                for (int i = 0; i < 7; ++i) sum += rf1.data[i];
                bool payload_ok = (sum == rf1.data[7]);
                for (int i = 2; i < 7 && payload_ok; ++i) {
                    if (rf1.data[i] != static_cast<uint8_t>((rf1.data[1] + i * 3) & 0xFF)) {
                        payload_ok = false;
                    }
                }
                if (payload_ok) {
                    struct can_frame pong{};
                    pong.can_id = 0x221;
                    pong.can_dlc = 8;
                    pong.data[0] = 0xC1;
                    pong.data[1] = rf1.data[1];
                    for (int i = 2; i < 7; ++i) {
                        pong.data[i] = static_cast<uint8_t>(rf1.data[i] ^ 0x5Au);
                    }
                    uint8_t psum = 0;
                    for (int i = 0; i < 7; ++i) psum += pong.data[i];
                    pong.data[7] = psum;
                    if (write(s1, &pong, sizeof(pong)) < 0) {}
                } else {
                    crc_errors.fetch_add(1);
                }
            }
        }

        while (true) {
            struct can_frame rf0{};
            ssize_t r = read(s0, &rf0, sizeof(rf0));
            if (r < static_cast<ssize_t>(sizeof(rf0))) break;
            if (rf0.can_id & CAN_ERR_FLAG) {
                bus_errors.fetch_add(1);
                continue;
            }
            if (rf0.can_id == 0x221 && rf0.can_dlc == 8 && rf0.data[0] == 0xC1) {
                uint8_t sum = 0;
                for (int i = 0; i < 7; ++i) sum += rf0.data[i];
                if (sum == rf0.data[7]) match_rx(rf0.data[1], rf0.data);
                else crc_errors.fetch_add(1);
            }
        }
    }
}

void CanBusTester::run() {
    std::string m0 = ensure_can_up(if0);
    ensure_can_up(if1);
    set_bitrate_info(m0);

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
    uint8_t tx_seq = 0;
    std::deque<PendingCanFrame> pending_tx;

    auto last_tx = std::chrono::steady_clock::now();
    auto last_calc = std::chrono::steady_clock::now();
    auto settle_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(15);
    bool in_settle = true;
    uint64_t tx_calc = 0;
    uint64_t rx_calc = 0;
    const uint64_t RX_TIMEOUT_US = 500000; // 500ms 判定超时丢帧

    CanDirection cur_dir = dir.load();

    while (active.load()) {
        CanDirection req_dir = dir.load();
        bool do_reset = reset_requested.exchange(false);
        if (req_dir != cur_dir || do_reset) {
            bool dir_changed = (req_dir != cur_dir);
            cur_dir = req_dir;
            flush_can_socket(s0);
            flush_can_socket(s1);
            pending_tx.clear();
            tx_seq = 0;
            tx_calc = 0;
            rx_calc = 0;
            clear_counters();
            auto tnow = std::chrono::steady_clock::now();
            last_calc = tnow;
            settle_until = tnow + std::chrono::milliseconds(dir_changed ? 30 : 15);
            in_settle = true;
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
            last_tx = now - std::chrono::milliseconds(20);
            last_calc = now;
        }

        uint64_t now_us = get_time_us();

        // 1. TX: 每 20ms (~50 fps)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tx).count() >= 20) {
            last_tx = now;

            if (cur_dir == CanDirection::CAN0_TO_CAN1 || cur_dir == CanDirection::PING_PONG) {
                struct can_frame f0{};
                f0.can_id = 0x121;
                f0.can_dlc = 8;
                f0.data[0] = 0xC0;
                f0.data[1] = tx_seq;
                for (int i = 2; i < 7; ++i) f0.data[i] = static_cast<uint8_t>((tx_seq + i * 3) & 0xFF);
                uint8_t sum = 0;
                for (int i = 0; i < 7; ++i) sum += f0.data[i];
                f0.data[7] = sum;

                ssize_t w = write(s0, &f0, sizeof(f0));
                if (w == static_cast<ssize_t>(sizeof(f0))) {
                    tx_frames.fetch_add(1);
                    tx_bytes.fetch_add(8);
                    last_tx_seq.store(tx_seq);
                    tx_calc++;
                    PendingCanFrame pf{};
                    pf.seq = tx_seq;
                    pf.send_time_us = now_us;
                    if (cur_dir == CanDirection::PING_PONG) {
                        pf.expect_data[0] = 0xC1;
                        pf.expect_data[1] = tx_seq;
                        for (int i = 2; i < 7; ++i) {
                            pf.expect_data[i] = static_cast<uint8_t>(f0.data[i] ^ 0x5Au);
                        }
                        uint8_t psum = 0;
                        for (int i = 0; i < 7; ++i) psum += pf.expect_data[i];
                        pf.expect_data[7] = psum;
                    } else {
                        std::memcpy(pf.expect_data, f0.data, 8);
                    }
                    pending_tx.push_back(pf);
                    tx_seq++;
                } else {
                    tx_frames.fetch_add(1);
                    err_frames.fetch_add(1);
                }
            } else if (cur_dir == CanDirection::CAN1_TO_CAN0) {
                struct can_frame f1{};
                f1.can_id = 0x221;
                f1.can_dlc = 8;
                f1.data[0] = 0xC1;
                f1.data[1] = tx_seq;
                for (int i = 2; i < 7; ++i) f1.data[i] = static_cast<uint8_t>((tx_seq + i * 5) & 0xFF);
                uint8_t sum = 0;
                for (int i = 0; i < 7; ++i) sum += f1.data[i];
                f1.data[7] = sum;

                ssize_t w = write(s1, &f1, sizeof(f1));
                if (w == static_cast<ssize_t>(sizeof(f1))) {
                    tx_frames.fetch_add(1);
                    tx_bytes.fetch_add(8);
                    last_tx_seq.store(tx_seq);
                    tx_calc++;
                    PendingCanFrame pf{};
                    pf.seq = tx_seq;
                    pf.send_time_us = now_us;
                    std::memcpy(pf.expect_data, f1.data, 8);
                    pending_tx.push_back(pf);
                    tx_seq++;
                } else {
                    tx_frames.fetch_add(1);
                    err_frames.fetch_add(1);
                }
            }
        }

        // 2. RX
        process_rx(s0, s1, cur_dir, pending_tx, rx_calc);

        // 3. 超时检测
        now_us = get_time_us();
        while (!pending_tx.empty() && (now_us - pending_tx.front().send_time_us) > RX_TIMEOUT_US) {
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

        struct pollfd pfds[2];
        pfds[0] = {s0, POLLIN, 0};
        pfds[1] = {s1, POLLIN, 0};
        int pr = poll(pfds, 2, 2);
        if (pr > 0 && !((pfds[0].revents & POLLIN) || (pfds[1].revents & POLLIN))) {
            usleep(2000);
        }
    }

    // 停止前短暂排空在途帧（500kbps 下单帧约 0.25ms，等待最多 30ms）
    auto drain_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(30);
    while (!pending_tx.empty() && std::chrono::steady_clock::now() < drain_end) {
        process_rx(s0, s1, cur_dir, pending_tx, rx_calc);
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
