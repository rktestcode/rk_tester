#include "serial_tester.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <termios.h>
#include <unistd.h>

SerialChannel::SerialChannel(const std::string& proto, const std::string& sa, const std::string& sb,
                             const std::string& dev_a, const std::string& dev_b, const std::string& key)
    : proto_name(proto), short_a(sa), short_b(sb), tx_dev(dev_a), rx_dev(dev_b), dir_key(key) {}

SerialChannel::~SerialChannel() {
    stop();
}

void SerialChannel::cycle_direction() {
    int d = (static_cast<int>(dir.load()) + 1) % 3;
    dir.store(static_cast<SerialDirection>(d));
    if (!active.load()) reset();
}

void SerialChannel::set_direction(SerialDirection d) {
    dir.store(d);
    if (!active.load()) reset();
}

std::string SerialChannel::get_short_label() const {
    switch (dir.load()) {
        case SerialDirection::A_TO_B:    return proto_name + "(" + short_a + "->" + short_b.substr(1) + "):";
        case SerialDirection::B_TO_A:    return proto_name + "(" + short_b + "->" + short_a.substr(1) + "):";
        case SerialDirection::PING_PONG: return proto_name + "(" + short_a + "<>" + short_b.substr(1) + "):";
    }
    return proto_name + "(" + short_a + "->" + short_b.substr(1) + "):";
}

std::string SerialChannel::get_wide_label() const {
    switch (dir.load()) {
        case SerialDirection::A_TO_B:    return proto_name + "(" + short_a + "->" + short_b + "):";
        case SerialDirection::B_TO_A:    return proto_name + "(" + short_b + "->" + short_a + "):";
        case SerialDirection::PING_PONG: return proto_name + "(" + short_a + "<>" + short_b + "):";
    }
    return proto_name + "(" + short_a + "->" + short_b + "):";
}

std::string SerialChannel::get_short_mode_name() const {
    switch (dir.load()) {
        case SerialDirection::A_TO_B:    return short_a + "->" + short_b;
        case SerialDirection::B_TO_A:    return short_b + "->" + short_a;
        case SerialDirection::PING_PONG: return short_a + "<->" + short_b + " PP";
    }
    return short_a + "->" + short_b;
}

std::string SerialChannel::get_mode_name() const {
    switch (dir.load()) {
        case SerialDirection::A_TO_B:    return proto_name + " (" + tx_dev + " -> " + rx_dev + " Uni)";
        case SerialDirection::B_TO_A:    return proto_name + " (" + rx_dev + " -> " + tx_dev + " Uni)";
        case SerialDirection::PING_PONG: return proto_name + " (" + tx_dev + " <-> " + rx_dev + " Ping-Pong)";
    }
    return proto_name + " (" + tx_dev + " -> " + rx_dev + " Uni)";
}

std::string SerialChannel::get_menu_desc() const {
    switch (dir.load()) {
        case SerialDirection::A_TO_B:    return short_a + "->" + short_b + " 115k Uni [" + dir_key + "]";
        case SerialDirection::B_TO_A:    return short_b + "->" + short_a + " 115k Uni [" + dir_key + "]";
        case SerialDirection::PING_PONG: return short_a + "<->" + short_b + " 115k PP [" + dir_key + "]";
    }
    return short_a + "->" + short_b + " 115k Uni [" + dir_key + "]";
}

std::string SerialChannel::get_error_detail() const {
    std::lock_guard<std::mutex> lk(err_mtx);
    return last_error_detail;
}

void SerialChannel::start() {
    if (active.load()) return;
    if (worker.joinable()) worker.join();
    reset_requested.store(false);
    set_error_detail("");
    state.store(ChannelState::RUNNING);
    active.store(true);
    worker = std::thread(&SerialChannel::run, this);
}

void SerialChannel::stop() {
    active.store(false);
    if (worker.joinable()) worker.join();
    speed_kb.store(0.0);
    tx_speed_kb.store(0.0);
    tx_fps.store(0.0);
    rx_fps.store(0.0);
    if (state.load() != ChannelState::PORT_ERROR) {
        state.store(ChannelState::STANDBY);
    }
}

void SerialChannel::reset() {
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

void SerialChannel::set_error_detail(const std::string& msg) {
    std::lock_guard<std::mutex> lk(err_mtx);
    last_error_detail = msg;
}

void SerialChannel::clear_counters() {
    tx_bytes.store(0);
    rx_bytes.store(0);
    tx_frames.store(0);
    rx_frames.store(0);
    err_frames.store(0);
    crc_errors.store(0);
    hw_oe.store(0);
    hw_fe.store(0);
    hw_pe.store(0);
    speed_kb.store(0.0);
    tx_speed_kb.store(0.0);
    tx_fps.store(0.0);
    rx_fps.store(0.0);
    rtt_cur_ms.store(0.0);
    rtt_max_ms.store(0.0);
    rtt_sum_ms.store(0.0);
    last_tx_seq.store(0);
    last_rx_seq.store(0);
    last_valid_rx_us.store(0);
}

int SerialChannel::open_port(const std::string& path, int& out_errno) {
    out_errno = 0;
    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        out_errno = errno;
        return -1;
    }
    struct termios tio{};
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CS8 | CREAD | CLOCAL);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        out_errno = errno;
        close(fd);
        return -1;
    }
    return fd;
}

bool SerialChannel::write_all_nonblock(int fd, const uint8_t* data, size_t len) {
    size_t sent = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(15);
    while (sent < len) {
        ssize_t w = write(fd, data + sent, len - sent);
        if (w > 0) {
            sent += static_cast<size_t>(w);
        } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            if (std::chrono::steady_clock::now() >= deadline) return false;
            usleep(500);
        } else {
            return false;
        }
    }
    return true;
}

SerialFrame SerialChannel::build_frame_from_payload(uint8_t s1, uint8_t s2, uint16_t seq, const uint8_t payload[58]) {
    SerialFrame frame{};
    frame.sync1 = s1;
    frame.sync2 = s2;
    frame.seq = seq;
    std::memcpy(frame.payload, payload, 58);
    uint16_t ck = 0;
    const auto* p = reinterpret_cast<const uint8_t*>(&frame);
    for (size_t i = 0; i < sizeof(SerialFrame) - 2; ++i) ck += p[i];
    frame.checksum = ck;
    return frame;
}

SerialFrame SerialChannel::build_frame(uint8_t s1, uint8_t s2, uint16_t seq, uint8_t salt) {
    uint8_t payload[58];
    for (int i = 0; i < 58; ++i) {
        payload[i] = static_cast<uint8_t>((i * salt + seq) & 0xFF);
    }
    return build_frame_from_payload(s1, s2, seq, payload);
}

void SerialChannel::drain_fd_into(int fd, std::vector<uint8_t>& buf, std::atomic<uint64_t>* byte_counter) {
    uint8_t tmp[256];
    while (true) {
        ssize_t r = read(fd, tmp, sizeof(tmp));
        if (r <= 0) break;
        if (byte_counter) byte_counter->fetch_add(static_cast<uint64_t>(r));
        buf.insert(buf.end(), tmp, tmp + r);
        if (buf.size() > 4096) {
            buf.erase(buf.begin(), buf.end() - 2048);
        }
    }
}

void SerialChannel::discard_fd(int fd) {
    uint8_t tmp[256];
    while (read(fd, tmp, sizeof(tmp)) > 0) {}
}

bool SerialChannel::read_uart_hw_errors(const std::string& dev_path, uint64_t& oe, uint64_t& fe, uint64_t& pe) {
    oe = 0;
    fe = 0;
    pe = 0;
    size_t pos = dev_path.rfind("ttyS");
    if (pos == std::string::npos) return false;
    std::string prefix = dev_path.substr(pos + 4) + ":";

    FILE* f = std::fopen("/proc/tty/driver/serial", "r");
    if (!f) return false;
    char line[256];
    bool found = false;
    while (std::fgets(line, sizeof(line), f)) {
        if (std::strncmp(line, prefix.c_str(), prefix.size()) == 0) {
            auto parse_tok = [&](const char* key) -> uint64_t {
                const char* p = std::strstr(line, key);
                if (!p) return 0;
                return static_cast<uint64_t>(std::strtoull(p + std::strlen(key), nullptr, 10));
            };
            oe = parse_tok("oe:");
            fe = parse_tok("fe:");
            pe = parse_tok("pe:");
            found = true;
            break;
        }
    }
    std::fclose(f);
    return found;
}

void SerialChannel::match_rx_stream(std::vector<uint8_t>& rx_buf, uint8_t exp_s1, uint8_t exp_s2,
                                    std::deque<PendingSerialFrame>& pending_tx,
                                    uint64_t& rx_bytes_since_calc, uint64_t& rx_frames_since_calc) {
    uint64_t now_us = get_time_us();
    while (rx_buf.size() >= sizeof(SerialFrame)) {
        if (rx_buf[0] == exp_s1 && rx_buf[1] == exp_s2) {
            SerialFrame f{};
            std::memcpy(&f, rx_buf.data(), sizeof(SerialFrame));
            uint16_t ck = 0;
            for (size_t i = 0; i < sizeof(SerialFrame) - 2; ++i) ck += rx_buf[i];

            if (ck == f.checksum) {
                uint16_t rx_seq = f.seq;
                auto it = std::find_if(pending_tx.begin(), pending_tx.end(),
                                       [&](const auto& item) { return item.seq == rx_seq; });
                if (it != pending_tx.end()) {
                    // 逐字节比对全部 58 字节载荷是否与发送数据完全一致
                    if (std::memcmp(f.payload, it->expect_payload, 58) == 0) {
                        double rtt_ms = (now_us >= it->send_time_us) ? ((now_us - it->send_time_us) / 1000.0) : 0.0;
                        pending_tx.erase(it);
                        rx_frames.fetch_add(1);
                        last_rx_seq.store(rx_seq);
                        last_valid_rx_us.store(now_us);
                        rtt_cur_ms.store(rtt_ms);
                        if (rtt_ms > rtt_max_ms.load()) rtt_max_ms.store(rtt_ms);
                        rtt_sum_ms.store(rtt_sum_ms.load() + rtt_ms);
                        rx_bytes_since_calc += sizeof(SerialFrame);
                        rx_frames_since_calc++;
                    } else {
                        pending_tx.erase(it);
                        crc_errors.fetch_add(1);
                        err_frames.fetch_add(1);
                    }
                }
                rx_buf.erase(rx_buf.begin(), rx_buf.begin() + sizeof(SerialFrame));
            } else {
                crc_errors.fetch_add(1);
                rx_buf.erase(rx_buf.begin());
            }
        } else {
            rx_buf.erase(rx_buf.begin());
        }
    }
}

void SerialChannel::process_rx(int fd_a, int fd_b, SerialDirection cur_dir,
                               std::vector<uint8_t>& rx_buf_a, std::vector<uint8_t>& rx_buf_b,
                               std::deque<PendingSerialFrame>& pending_tx,
                               uint64_t& rx_bytes_since_calc, uint64_t& rx_frames_since_calc) {
    if (cur_dir == SerialDirection::A_TO_B) {
        discard_fd(fd_a); // 排空发送端可能产生的半双工回显/残留字节，避免 poll 忙等
        drain_fd_into(fd_b, rx_buf_b, &rx_bytes);
        match_rx_stream(rx_buf_b, 0xAA, 0x55, pending_tx, rx_bytes_since_calc, rx_frames_since_calc);
    } else if (cur_dir == SerialDirection::B_TO_A) {
        discard_fd(fd_b); // 排空发送端可能产生的半双工回显/残留字节，避免 poll 忙等
        drain_fd_into(fd_a, rx_buf_a, &rx_bytes);
        match_rx_stream(rx_buf_a, 0x55, 0xAA, pending_tx, rx_bytes_since_calc, rx_frames_since_calc);
    } else {
        // PING_PONG: Port B 逐字节校验收到的 Ping 载荷后取反回发 Pong，Port A 再做全载荷一致性校验
        drain_fd_into(fd_b, rx_buf_b, nullptr);
        while (rx_buf_b.size() >= sizeof(SerialFrame)) {
            if (rx_buf_b[0] == 0xAA && rx_buf_b[1] == 0x55) {
                SerialFrame f{};
                std::memcpy(&f, rx_buf_b.data(), sizeof(SerialFrame));
                uint16_t ck = 0;
                for (size_t i = 0; i < sizeof(SerialFrame) - 2; ++i) ck += rx_buf_b[i];
                if (ck == f.checksum) {
                    uint16_t ping_seq = f.seq;
                    SerialFrame exp_ping = build_frame(0xAA, 0x55, ping_seq, 1);
                    if (std::memcmp(f.payload, exp_ping.payload, 58) == 0) {
                        uint8_t pong_payload[58];
                        for (int i = 0; i < 58; ++i) {
                            pong_payload[i] = static_cast<uint8_t>(~f.payload[i]);
                        }
                        SerialFrame pong = build_frame_from_payload(0x55, 0xAA, ping_seq, pong_payload);
                        write_all_nonblock(fd_b, reinterpret_cast<const uint8_t*>(&pong), sizeof(pong));
                    } else {
                        crc_errors.fetch_add(1);
                    }
                    rx_buf_b.erase(rx_buf_b.begin(), rx_buf_b.begin() + sizeof(SerialFrame));
                } else {
                    crc_errors.fetch_add(1);
                    rx_buf_b.erase(rx_buf_b.begin());
                }
            } else {
                rx_buf_b.erase(rx_buf_b.begin());
            }
        }

        drain_fd_into(fd_a, rx_buf_a, &rx_bytes);
        match_rx_stream(rx_buf_a, 0x55, 0xAA, pending_tx, rx_bytes_since_calc, rx_frames_since_calc);
    }
}

void SerialChannel::run() {
    // 提升为低层级实时优先级（SCHED_FIFO 20，低于内核中断线程 FIFO 50，高于所有普通烤机线程），
    // 消除满载 CPU 竞争引起的串口用户态唤醒抖动与 RTT 尖峰
    struct sched_param rt_sp{};
    rt_sp.sched_priority = 20;
    int rt_pol = SCHED_FIFO;
#ifdef SCHED_RESET_ON_FORK
    rt_pol |= SCHED_RESET_ON_FORK;
#endif
    if (pthread_setschedparam(pthread_self(), rt_pol, &rt_sp) != 0) {
        setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), -10);
    }

    int err_a = 0, err_b = 0;
    int fd_a = open_port(tx_dev, err_a);
    int fd_b = open_port(rx_dev, err_b);
    if (fd_a < 0 || fd_b < 0) {
        std::string detail;
        if (fd_a < 0) detail = tx_dev + " (" + std::strerror(err_a) + ")";
        else detail = rx_dev + " (" + std::strerror(err_b) + ")";
        set_error_detail(detail);
        if (fd_a >= 0) close(fd_a);
        if (fd_b >= 0) close(fd_b);
        state.store(ChannelState::PORT_ERROR);
        active.store(false);
        return;
    }

    tcflush(fd_a, TCIOFLUSH);
    tcflush(fd_b, TCIOFLUSH);
    discard_fd(fd_a);
    discard_fd(fd_b);
    state.store(ChannelState::RUNNING);

    uint16_t cur_seq = 1;
    std::vector<uint8_t> rx_buf_a, rx_buf_b;
    rx_buf_a.reserve(2048);
    rx_buf_b.reserve(2048);
    std::deque<PendingSerialFrame> pending_tx;

    auto next_tx = std::chrono::steady_clock::now();
    auto last_calc = std::chrono::steady_clock::now();
    auto last_hw_poll = std::chrono::steady_clock::now();
    auto settle_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    bool in_settle = true;
    bool rx_ready = true;
    uint64_t tx_bytes_since_calc = 0;
    uint64_t tx_frames_since_calc = 0;
    uint64_t rx_bytes_since_calc = 0;
    uint64_t rx_frames_since_calc = 0;
    const uint64_t RX_TIMEOUT_US = 500000; // 500ms 判定超时丢帧

    uint64_t base_oe = 0, base_fe = 0, base_pe = 0;
    auto sample_hw_sum = [&](uint64_t& sum_oe, uint64_t& sum_fe, uint64_t& sum_pe) {
        uint64_t oe_a = 0, fe_a = 0, pe_a = 0, oe_b = 0, fe_b = 0, pe_b = 0;
        read_uart_hw_errors(tx_dev, oe_a, fe_a, pe_a);
        read_uart_hw_errors(rx_dev, oe_b, fe_b, pe_b);
        sum_oe = oe_a + oe_b;
        sum_fe = fe_a + fe_b;
        sum_pe = pe_a + pe_b;
    };
    sample_hw_sum(base_oe, base_fe, base_pe);

    SerialDirection cur_dir = dir.load();

    while (active.load()) {
        SerialDirection req_dir = dir.load();
        bool do_reset = reset_requested.exchange(false); // 避免 || 短路遗漏清零标志
        if (req_dir != cur_dir || do_reset) {
            bool dir_changed = (req_dir != cur_dir);
            cur_dir = req_dir;
            tcflush(fd_a, TCIOFLUSH);
            tcflush(fd_b, TCIOFLUSH);
            discard_fd(fd_a);
            discard_fd(fd_b);
            rx_buf_a.clear();
            rx_buf_b.clear();
            pending_tx.clear();
            cur_seq = 1;
            tx_bytes_since_calc = 0;
            tx_frames_since_calc = 0;
            rx_bytes_since_calc = 0;
            rx_frames_since_calc = 0;
            clear_counters();
            sample_hw_sum(base_oe, base_fe, base_pe);
            auto tnow = std::chrono::steady_clock::now();
            last_calc = tnow;
            last_hw_poll = tnow;
            // 切方向后进入 60ms 静默排空窗口，等 UART 硬件 TX FIFO 与总线残留彻底结束再发新帧
            settle_until = tnow + std::chrono::milliseconds(dir_changed ? 60 : 25);
            in_settle = true;
            rx_ready = true;
        }

        auto now = std::chrono::steady_clock::now();
        if (in_settle) {
            discard_fd(fd_a);
            discard_fd(fd_b);
            rx_buf_a.clear();
            rx_buf_b.clear();
            if (now < settle_until) {
                usleep(2000);
                continue;
            }
            tcflush(fd_a, TCIOFLUSH);
            tcflush(fd_b, TCIOFLUSH);
            discard_fd(fd_a);
            discard_fd(fd_b);
            sample_hw_sum(base_oe, base_fe, base_pe);
            in_settle = false;
            next_tx = now;
            last_calc = now;
            last_hw_poll = now;
            rx_ready = true;
        }

        uint64_t now_us = get_time_us();

        // 单向每 20ms 固定相位发一帧 (50F/s)；Ping-Pong 半双工每 30ms 发一轮，确保 RS485 方向切换无冲突
        int tx_interval_ms = (cur_dir == SerialDirection::PING_PONG) ? 30 : 20;
        const auto tx_period = std::chrono::milliseconds(tx_interval_ms);
        if (now >= next_tx) {
            next_tx += tx_period;
            if (next_tx <= now - tx_period) {
                next_tx = now + tx_period;
            }
            uint16_t tx_seq = cur_seq++;
            SerialFrame frame = (cur_dir == SerialDirection::B_TO_A)
                                    ? build_frame(0x55, 0xAA, tx_seq, 2)
                                    : build_frame(0xAA, 0x55, tx_seq, 1);
            int send_fd = (cur_dir == SerialDirection::B_TO_A) ? fd_b : fd_a;

            if (write_all_nonblock(send_fd, reinterpret_cast<const uint8_t*>(&frame), sizeof(frame))) {
                tx_bytes.fetch_add(sizeof(frame));
                tx_frames.fetch_add(1);
                last_tx_seq.store(tx_seq);
                tx_bytes_since_calc += sizeof(frame);
                tx_frames_since_calc++;
                PendingSerialFrame pf{};
                pf.seq = tx_seq;
                pf.send_time_us = now_us;
                if (cur_dir == SerialDirection::PING_PONG) {
                    for (int i = 0; i < 58; ++i) {
                        pf.expect_payload[i] = static_cast<uint8_t>(~frame.payload[i]);
                    }
                } else {
                    std::memcpy(pf.expect_payload, frame.payload, 58);
                }
                pending_tx.push_back(pf);
                if (cur_dir == SerialDirection::A_TO_B) discard_fd(fd_a);
                else if (cur_dir == SerialDirection::B_TO_A) discard_fd(fd_b);
            } else {
                tx_frames.fetch_add(1);
                err_frames.fetch_add(1);
            }
        }

        // 2. RX（仅在 poll 提示可读时执行系统调用，避免空读）
        if (rx_ready) {
            process_rx(fd_a, fd_b, cur_dir, rx_buf_a, rx_buf_b, pending_tx,
                       rx_bytes_since_calc, rx_frames_since_calc);
            rx_ready = false;
        }

        // 3. 超时丢帧检测
        now_us = get_time_us();
        while (!pending_tx.empty() && (now_us - pending_tx.front().send_time_us) > RX_TIMEOUT_US) {
            pending_tx.pop_front();
            err_frames.fetch_add(1);
        }

        // 4. 每 500ms 刷新实时收发吞吐与帧率
        auto calc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calc).count();
        if (calc_ms >= 500) {
            double dt = calc_ms / 1000.0;
            last_calc = now;
            if (dt <= 0.0) dt = 0.5;
            speed_kb.store((rx_bytes_since_calc / 1024.0) / dt);
            tx_speed_kb.store((tx_bytes_since_calc / 1024.0) / dt);
            tx_fps.store(tx_frames_since_calc / dt);
            rx_fps.store(rx_frames_since_calc / dt);
            tx_bytes_since_calc = 0;
            tx_frames_since_calc = 0;
            rx_bytes_since_calc = 0;
            rx_frames_since_calc = 0;
        }

        // 5. 每 1000ms 采样一次内核 UART 硬件错误计数 (oe/fe/pe)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_hw_poll).count() >= 1000) {
            last_hw_poll = now;
            uint64_t cur_oe = 0, cur_fe = 0, cur_pe = 0;
            sample_hw_sum(cur_oe, cur_fe, cur_pe);
            hw_oe.store(cur_oe >= base_oe ? (cur_oe - base_oe) : 0);
            hw_fe.store(cur_fe >= base_fe ? (cur_fe - base_fe) : 0);
            hw_pe.store(cur_pe >= base_pe ? (cur_pe - base_pe) : 0);
        }

        // 纳秒级 ppoll() 精确休眠至下一固定发包相位，消除毫秒取整漂移；有串口数据到达立即唤醒
        auto after = std::chrono::steady_clock::now();
        long long rem_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(next_tx - after).count();
        if (rem_ns < 0) rem_ns = 0;
        if (rem_ns > 50000000LL) rem_ns = 50000000LL;
        struct timespec ts{};
        ts.tv_sec = static_cast<time_t>(rem_ns / 1000000000LL);
        ts.tv_nsec = static_cast<long>(rem_ns % 1000000000LL);

        struct pollfd pfds[2];
        nfds_t nfds = 0;
        if (cur_dir == SerialDirection::A_TO_B) {
            pfds[nfds++] = {fd_b, POLLIN, 0};
        } else if (cur_dir == SerialDirection::B_TO_A) {
            pfds[nfds++] = {fd_a, POLLIN, 0};
        } else {
            pfds[nfds++] = {fd_a, POLLIN, 0};
            pfds[nfds++] = {fd_b, POLLIN, 0};
        }
        int pr = ppoll(pfds, nfds, &ts, nullptr);
        if (pr > 0) {
            bool got_in = false;
            for (nfds_t i = 0; i < nfds; ++i) {
                if (pfds[i].revents & POLLIN) got_in = true;
            }
            if (got_in) rx_ready = true;
            else usleep(2000);
        }
    }

    // 退出前短暂排空在途帧（等待最多 50ms，覆盖 Ping-Pong 往返 ~12ms）
    auto drain_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    while (!pending_tx.empty() && std::chrono::steady_clock::now() < drain_end) {
        process_rx(fd_a, fd_b, cur_dir, rx_buf_a, rx_buf_b, pending_tx,
                   rx_bytes_since_calc, rx_frames_since_calc);
        if (!pending_tx.empty()) usleep(2000);
    }
    if (!pending_tx.empty()) {
        err_frames.fetch_add(pending_tx.size());
        pending_tx.clear();
    }

    close(fd_a);
    close(fd_b);
    speed_kb.store(0.0);
    tx_speed_kb.store(0.0);
    tx_fps.store(0.0);
    rx_fps.store(0.0);
    state.store(ChannelState::STANDBY);
}
