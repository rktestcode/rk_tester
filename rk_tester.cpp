#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <tuple>
#include <thread>
#include <atomic>
#include <chrono>
#include <regex>
#include <cmath>
#include <cstring>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <pthread.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <clocale>
#include <termios.h>
#include <poll.h>
#include <ncurses.h>

// 硬件路径定义
static const std::string DEVFREQ_GPU = "/sys/class/devfreq/27800000.gpu";
static const std::string DEVFREQ_NPU = "/sys/class/devfreq/27700000.npu";
static const std::string DEBUG_RKNPU_LOAD = "/sys/kernel/debug/rknpu/load";
static const std::string RKNN_TEST_BIN = "/usr/bin/rknn_common_test";
static const std::string RKNN_MODEL = "/usr/share/model/RK3576/mobilenet_v1.rknn";
static const std::string RKNN_IMG = "/usr/share/model/dog_224x224.jpg";

// ICMP 校验和计算
static uint16_t in_cksum(const uint16_t *addr, int len) {
    int nleft = len;
    const uint16_t *w = addr;
    int sum = 0;
    uint16_t answer = 0;
    while (nleft > 1) {
        sum += *w++;
        nleft -= 2;
    }
    if (nleft == 1) {
        *(unsigned char *)(&answer) = *(const unsigned char *)w;
        sum += answer;
    }
    sum = (sum >> 16) + (sum & 0xffff);
    sum += (sum >> 16);
    answer = ~sum;
    return answer;
}

static inline uint64_t get_time_us() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

static inline std::string get_time_str() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

struct CpuTime {
    unsigned long long total{0};
    unsigned long long idle{0};
};

struct SentPacket {
    uint64_t send_time_us{0};
    bool replied{false};
    bool marked_lost{false};
};

struct NetPortInfo {
    std::string name;       // eth0, eth1, eth2, eth3
    std::string ch_name;    // Port 1 (USB/eth0)
    std::string target_ip;  // 192.168.32.101 ~ 104
    std::string current_ip; // Current IP
    bool carrier{false};    // Link carrier up/down
    int speed{-1};          // Speed (1000, 100, 10)

    // Per-port independent raw socket and EMC stats
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
};

struct SerialFrame {
    uint8_t sync1{0xAA};
    uint8_t sync2{0x55};
    uint16_t seq{0};
    uint8_t payload[58];
    uint16_t checksum{0};
} __attribute__((packed));

class SerialChannel {
public:
    std::string name;
    std::string tx_dev;
    std::string rx_dev;
    std::atomic<bool> active{false};

    std::atomic<uint64_t> tx_bytes{0};
    std::atomic<uint64_t> rx_bytes{0};
    std::atomic<uint64_t> tx_frames{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> err_frames{0};
    std::atomic<double> speed_kb{0.0};
    std::string status_msg{"STANDBY"};

    std::thread worker;

    SerialChannel(const std::string& n, const std::string& tx, const std::string& rx)
        : name(n), tx_dev(tx), rx_dev(rx) {}

    ~SerialChannel() {
        stop();
    }

    void start() {
        if (active.load()) return;
        if (worker.joinable()) worker.join();
        active.store(true);
        worker = std::thread(&SerialChannel::run, this);
    }

    void stop() {
        active.store(false);
        if (worker.joinable()) worker.join();
        speed_kb.store(0.0);
        status_msg = "STANDBY";
    }

    void reset() {
        tx_bytes.store(0);
        rx_bytes.store(0);
        tx_frames.store(0);
        rx_frames.store(0);
        err_frames.store(0);
        speed_kb.store(0.0);
    }

private:
    int open_port(const std::string& path) {
        int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return -1;
        struct termios tio;
        std::memset(&tio, 0, sizeof(tio));
        cfmakeraw(&tio);
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tio.c_cflag |= (CS8 | CREAD | CLOCAL);
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        tcflush(fd, TCIOFLUSH);
        tcsetattr(fd, TCSANOW, &tio);
        return fd;
    }

    void run() {
        int tx_fd = open_port(tx_dev);
        int rx_fd = open_port(rx_dev);
        if (tx_fd < 0 || rx_fd < 0) {
            if (tx_fd >= 0) close(tx_fd);
            if (rx_fd >= 0) close(rx_fd);
            status_msg = "PORT ERROR";
            active.store(false);
            return;
        }

        tcflush(tx_fd, TCIOFLUSH);
        tcflush(rx_fd, TCIOFLUSH);

        status_msg = "RUNNING";
        uint16_t cur_seq = 0;
        uint16_t expected_rx_seq = 0;
        bool first_rx = true;

        std::vector<uint8_t> rx_buf;
        rx_buf.reserve(2048);

        auto last_send = std::chrono::steady_clock::now();
        auto last_calc = std::chrono::steady_clock::now();
        uint64_t bytes_since_calc = 0;

        while (active.load()) {
            auto now = std::chrono::steady_clock::now();

            // 1. TX: Send 64-byte frame every 20ms (~3.2 KB/s at 115200bps)
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() >= 20) {
                last_send = now;
                SerialFrame frame;
                frame.sync1 = 0xAA;
                frame.sync2 = 0x55;
                frame.seq = cur_seq++;
                for (int i = 0; i < 58; ++i) {
                    frame.payload[i] = (uint8_t)((i + frame.seq) & 0xFF);
                }
                uint16_t ck = 0;
                uint8_t* p = (uint8_t*)&frame;
                for (size_t i = 0; i < sizeof(SerialFrame) - 2; ++i) ck += p[i];
                frame.checksum = ck;

                ssize_t w = write(tx_fd, &frame, sizeof(frame));
                if (w > 0) {
                    tx_bytes.fetch_add(w);
                    tx_frames.fetch_add(1);
                    bytes_since_calc += w;
                }
            }

            // 2. RX: Read incoming bytes
            uint8_t tmp[256];
            ssize_t r = read(rx_fd, tmp, sizeof(tmp));
            if (r > 0) {
                rx_bytes.fetch_add(r);
                rx_buf.insert(rx_buf.end(), tmp, tmp + r);

                while (rx_buf.size() >= sizeof(SerialFrame)) {
                    if (rx_buf[0] == 0xAA && rx_buf[1] == 0x55) {
                        SerialFrame* f = (SerialFrame*)rx_buf.data();
                        uint16_t ck = 0;
                        uint8_t* p = rx_buf.data();
                        for (size_t i = 0; i < sizeof(SerialFrame) - 2; ++i) ck += p[i];

                        if (ck == f->checksum) {
                            rx_frames.fetch_add(1);
                            if (first_rx) {
                                expected_rx_seq = f->seq + 1;
                                first_rx = false;
                            } else {
                                if (f->seq != expected_rx_seq) {
                                    uint16_t diff = f->seq - expected_rx_seq;
                                    err_frames.fetch_add(diff);
                                    expected_rx_seq = f->seq + 1;
                                } else {
                                    expected_rx_seq++;
                                }
                            }
                            rx_buf.erase(rx_buf.begin(), rx_buf.begin() + sizeof(SerialFrame));
                        } else {
                            err_frames.fetch_add(1);
                            rx_buf.erase(rx_buf.begin());
                        }
                    } else {
                        rx_buf.erase(rx_buf.begin());
                    }
                }
            }

            // 3. Speed calculation every 500ms
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calc).count() >= 500) {
                double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calc).count() / 1000.0;
                last_calc = now;
                double kb_s = (bytes_since_calc / 1024.0) / (dt > 0 ? dt : 0.5);
                speed_kb.store(kb_s);
                bytes_since_calc = 0;
            }

            usleep(2000);
        }

        close(tx_fd);
        close(rx_fd);
        status_msg = "STANDBY";
    }
};

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
    std::atomic<CanDirection> dir{CanDirection::CAN0_TO_CAN1};

    std::atomic<uint64_t> tx_frames{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> err_frames{0};

    std::atomic<double> tx_fps{0.0};
    std::atomic<double> rx_fps{0.0};
    std::string status_msg{"STANDBY"};

    std::thread worker;

    CanBusTester() = default;
    ~CanBusTester() {
        stop();
    }

    void cycle_direction() {
        int d = (static_cast<int>(dir.load()) + 1) % 3;
        dir.store(static_cast<CanDirection>(d));
        reset();
    }

    void set_direction(CanDirection d) {
        dir.store(d);
        reset();
    }

    std::string get_mode_name() const {
        switch (dir.load()) {
            case CanDirection::CAN0_TO_CAN1: return "CAN0 -> CAN1 (Uni)";
            case CanDirection::CAN1_TO_CAN0: return "CAN1 -> CAN0 (Uni)";
            case CanDirection::PING_PONG:    return "CAN0 <-> CAN1 (Ping-Pong)";
        }
        return "CAN0 -> CAN1 (Uni)";
    }

    std::string get_short_mode_name() const {
        switch (dir.load()) {
            case CanDirection::CAN0_TO_CAN1: return "0->1";
            case CanDirection::CAN1_TO_CAN0: return "1->0";
            case CanDirection::PING_PONG:    return "0<->1 PP";
        }
        return "0->1";
    }

    void start() {
        if (active.load()) return;
        if (worker.joinable()) worker.join();
        active.store(true);
        status_msg = "RUNNING";
        worker = std::thread(&CanBusTester::run, this);
    }

    void stop() {
        active.store(false);
        if (worker.joinable()) worker.join();
        tx_fps.store(0.0);
        rx_fps.store(0.0);
        status_msg = "STANDBY";
    }

    void reset() {
        tx_frames.store(0);
        rx_frames.store(0);
        err_frames.store(0);
        tx_fps.store(0.0);
        rx_fps.store(0.0);
    }

    static void ensure_can_up(const std::string& dev) {
        std::ifstream f("/sys/class/net/" + dev + "/operstate");
        std::string state;
        if (f.is_open()) f >> state;
        if (state != "up") {
            std::string cmd = "ip link set " + dev + " type can bitrate 500000 dbitrate 2000000 fd on 2>/dev/null; "
                              "ip link set " + dev + " txqueuelen 1000 2>/dev/null; "
                              "ip link set " + dev + " up 2>/dev/null";
            if (system(cmd.c_str()) < 0) {}
        } else {
            std::string cmd = "ip link set " + dev + " txqueuelen 1000 2>/dev/null";
            if (system(cmd.c_str()) < 0) {}
        }
    }

    int open_socket(const std::string& ifname, canid_t filter_id) {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) return -1;
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { close(s); return -1; }
        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(s); return -1; }

        fcntl(s, F_SETFL, O_NONBLOCK);

        struct can_filter rfilter[1];
        rfilter[0].can_id = filter_id;
        rfilter[0].can_mask = CAN_SFF_MASK;
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

        // Flush any stale frames
        struct can_frame stale;
        while (read(s, &stale, sizeof(stale)) > 0) {}

        return s;
    }

    void run() {
        ensure_can_up(if0);
        ensure_can_up(if1);

        int s0 = open_socket(if0, 0x221); // s0 listens on can0 for 0x221
        int s1 = open_socket(if1, 0x121); // s1 listens on can1 for 0x121

        if (s0 < 0 || s1 < 0) {
            status_msg = "CAN_ERR: OPEN_FAIL";
            if (s0 >= 0) close(s0);
            if (s1 >= 0) close(s1);
            active.store(false);
            return;
        }

        uint8_t tx_seq = 0;
        uint8_t exp_rx_seq = 0;
        bool first_rx = true;

        auto last_tx = std::chrono::steady_clock::now();
        auto last_calc = std::chrono::steady_clock::now();
        uint64_t tx_calc = 0;
        uint64_t rx_calc = 0;

        CanDirection cur_dir = dir.load();

        while (active.load()) {
            auto now = std::chrono::steady_clock::now();

            if (dir.load() != cur_dir) {
                cur_dir = dir.load();
                tx_seq = 0;
                exp_rx_seq = 0;
                first_rx = true;
            }

            // 1. TX: every 20ms (~50 frames/s)
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tx).count() >= 20) {
                last_tx = now;

                if (cur_dir == CanDirection::CAN0_TO_CAN1 || cur_dir == CanDirection::PING_PONG) {
                    struct can_frame f0;
                    memset(&f0, 0, sizeof(f0));
                    f0.can_id = 0x121;
                    f0.can_dlc = 8;
                    f0.data[0] = 0xC0;
                    f0.data[1] = tx_seq;
                    for (int i = 2; i < 7; ++i) f0.data[i] = (uint8_t)((tx_seq + i * 3) & 0xFF);
                    uint8_t sum = 0;
                    for (int i = 0; i < 7; ++i) sum += f0.data[i];
                    f0.data[7] = sum;

                    ssize_t w = write(s0, &f0, sizeof(f0));
                    if (w == sizeof(f0)) {
                        tx_frames.fetch_add(1);
                        tx_calc++;
                        tx_seq++;
                    } else {
                        err_frames.fetch_add(1);
                    }
                } else if (cur_dir == CanDirection::CAN1_TO_CAN0) {
                    struct can_frame f1;
                    memset(&f1, 0, sizeof(f1));
                    f1.can_id = 0x221;
                    f1.can_dlc = 8;
                    f1.data[0] = 0xC1;
                    f1.data[1] = tx_seq;
                    for (int i = 2; i < 7; ++i) f1.data[i] = (uint8_t)((tx_seq + i * 5) & 0xFF);
                    uint8_t sum = 0;
                    for (int i = 0; i < 7; ++i) sum += f1.data[i];
                    f1.data[7] = sum;

                    ssize_t w = write(s1, &f1, sizeof(f1));
                    if (w == sizeof(f1)) {
                        tx_frames.fetch_add(1);
                        tx_calc++;
                        tx_seq++;
                    } else {
                        err_frames.fetch_add(1);
                    }
                }
            }

            // 2. RX Handling
            if (cur_dir == CanDirection::CAN0_TO_CAN1) {
                while (true) {
                    struct can_frame rf1;
                    ssize_t r = read(s1, &rf1, sizeof(rf1));
                    if (r < (ssize_t)sizeof(rf1)) break;

                    if (rf1.can_id == 0x121 && rf1.can_dlc == 8 && rf1.data[0] == 0xC0) {
                        uint8_t sum = 0;
                        for (int i = 0; i < 7; ++i) sum += rf1.data[i];
                        if (sum == rf1.data[7]) {
                            rx_frames.fetch_add(1);
                            rx_calc++;
                            if (first_rx) {
                                exp_rx_seq = (uint8_t)(rf1.data[1] + 1);
                                first_rx = false;
                            } else {
                                if (rf1.data[1] != exp_rx_seq) {
                                    uint8_t diff = (uint8_t)(rf1.data[1] - exp_rx_seq);
                                    err_frames.fetch_add(diff);
                                    exp_rx_seq = (uint8_t)(rf1.data[1] + 1);
                                } else {
                                    exp_rx_seq++;
                                }
                            }
                        } else {
                            err_frames.fetch_add(1);
                        }
                    }
                }
            } else if (cur_dir == CanDirection::CAN1_TO_CAN0) {
                while (true) {
                    struct can_frame rf0;
                    ssize_t r = read(s0, &rf0, sizeof(rf0));
                    if (r < (ssize_t)sizeof(rf0)) break;

                    if (rf0.can_id == 0x221 && rf0.can_dlc == 8 && rf0.data[0] == 0xC1) {
                        uint8_t sum = 0;
                        for (int i = 0; i < 7; ++i) sum += rf0.data[i];
                        if (sum == rf0.data[7]) {
                            rx_frames.fetch_add(1);
                            rx_calc++;
                            if (first_rx) {
                                exp_rx_seq = (uint8_t)(rf0.data[1] + 1);
                                first_rx = false;
                            } else {
                                if (rf0.data[1] != exp_rx_seq) {
                                    uint8_t diff = (uint8_t)(rf0.data[1] - exp_rx_seq);
                                    err_frames.fetch_add(diff);
                                    exp_rx_seq = (uint8_t)(rf0.data[1] + 1);
                                } else {
                                    exp_rx_seq++;
                                }
                            }
                        } else {
                            err_frames.fetch_add(1);
                        }
                    }
                }
            } else if (cur_dir == CanDirection::PING_PONG) {
                while (true) {
                    struct can_frame rf1;
                    ssize_t r = read(s1, &rf1, sizeof(rf1));
                    if (r < (ssize_t)sizeof(rf1)) break;

                    if (rf1.can_id == 0x121 && rf1.can_dlc == 8 && rf1.data[0] == 0xC0) {
                        uint8_t sum = 0;
                        for (int i = 0; i < 7; ++i) sum += rf1.data[i];
                        if (sum == rf1.data[7]) {
                            struct can_frame pong;
                            memset(&pong, 0, sizeof(pong));
                            pong.can_id = 0x221;
                            pong.can_dlc = 8;
                            pong.data[0] = 0xC1;
                            pong.data[1] = rf1.data[1];
                            for (int i = 2; i < 7; ++i) pong.data[i] = (uint8_t)((pong.data[1] + i * 7) & 0xFF);
                            uint8_t psum = 0;
                            for (int i = 0; i < 7; ++i) psum += pong.data[i];
                            pong.data[7] = psum;
                            if (write(s1, &pong, sizeof(pong)) < 0) {}
                        } else {
                            err_frames.fetch_add(1);
                        }
                    }
                }

                while (true) {
                    struct can_frame rf0;
                    ssize_t r = read(s0, &rf0, sizeof(rf0));
                    if (r < (ssize_t)sizeof(rf0)) break;

                    if (rf0.can_id == 0x221 && rf0.can_dlc == 8 && rf0.data[0] == 0xC1) {
                        uint8_t sum = 0;
                        for (int i = 0; i < 7; ++i) sum += rf0.data[i];
                        if (sum == rf0.data[7]) {
                            rx_frames.fetch_add(1);
                            rx_calc++;
                            if (first_rx) {
                                exp_rx_seq = (uint8_t)(rf0.data[1] + 1);
                                first_rx = false;
                            } else {
                                if (rf0.data[1] != exp_rx_seq) {
                                    uint8_t diff = (uint8_t)(rf0.data[1] - exp_rx_seq);
                                    err_frames.fetch_add(diff);
                                    exp_rx_seq = (uint8_t)(rf0.data[1] + 1);
                                } else {
                                    exp_rx_seq++;
                                }
                            }
                        } else {
                            err_frames.fetch_add(1);
                        }
                    }
                }
            }

            // 3. FPS calculation every 500ms
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calc).count() >= 500) {
                double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calc).count() / 1000.0;
                last_calc = now;
                if (dt <= 0) dt = 0.5;
                tx_fps.store(tx_calc / dt);
                rx_fps.store(rx_calc / dt);
                tx_calc = 0;
                rx_calc = 0;
            }

            usleep(2000);
        }

        close(s0);
        close(s1);
        status_msg = "STANDBY";
    }
};

struct UsbDiskInfo {
    std::string dev_name;
    std::string part_name;
    std::string mount_point;
    std::string label;
    std::string fs_type;
    std::string model;
    double capacity_gb{0.0};
    bool mounted{false};

    std::atomic<uint64_t> write_bytes{0};
    std::atomic<uint64_t> read_bytes{0};
    std::atomic<uint64_t> write_ops{0};
    std::atomic<uint64_t> read_ops{0};
    std::atomic<uint64_t> errors{0};
    std::atomic<uint64_t> disconnects{0};
    std::atomic<uint64_t> recovers{0};
    std::atomic<double> write_speed_mb{0.0};
    std::atomic<double> read_speed_mb{0.0};

    UsbDiskInfo(const std::string& dev, const std::string& part, const std::string& mnt,
                const std::string& lbl, const std::string& fs)
        : dev_name(dev), part_name(part), mount_point(mnt), label(lbl), fs_type(fs) {}
};

class UsbStorageTester {
public:
    std::vector<std::unique_ptr<UsbDiskInfo>> disks;
    std::atomic<bool> active{false};
    std::vector<std::thread> workers;

    UsbStorageTester() {
        init_disks();
    }

    ~UsbStorageTester() {
        stop();
    }

    bool is_mounted(const std::string& mount_point) {
        std::ifstream f("/proc/mounts");
        if (!f.is_open()) return false;
        std::string line;
        while (std::getline(f, line)) {
            std::istringstream ss(line);
            std::string dev, mnt;
            if (ss >> dev >> mnt) {
                if (mnt == mount_point) return true;
            }
        }
        return false;
    }

    std::string find_disk_partition(const std::string& dev_name) {
        std::string best_part = "";
        long long max_size = 0;
        // Check sysfs partition sizes (1..8) to select the largest data partition
        // Avoid selecting 16MB EFI or Microsoft Reserved partitions (e.g. sda1 vs sda2)
        for (int p = 1; p <= 8; ++p) {
            std::string part_name = dev_name + std::to_string(p);
            std::string size_file = "/sys/block/" + dev_name + "/" + part_name + "/size";
            std::ifstream sf(size_file);
            if (sf.is_open()) {
                long long sz = 0;
                if (sf >> sz && sz > max_size) {
                    max_size = sz;
                    best_part = "/dev/" + part_name;
                }
            }
        }
        if (!best_part.empty()) return best_part;

        // Fallback: check if raw block device exists (unpartitioned disk)
        if (access(("/dev/" + dev_name).c_str(), F_OK) == 0) {
            return "/dev/" + dev_name;
        }
        return "";
    }

    void read_disk_sysfs(UsbDiskInfo& d) {
        std::ifstream fm("/sys/block/" + d.dev_name + "/device/model");
        if (fm.is_open()) {
            std::getline(fm, d.model);
            while (!d.model.empty() && (d.model.back() == ' ' || d.model.back() == '\r')) d.model.pop_back();
        }
        std::ifstream fs("/sys/block/" + d.dev_name + "/size");
        long long sec = 0;
        if (fs.is_open() && (fs >> sec)) {
            d.capacity_gb = (double)sec * 512.0 / (1024.0 * 1024.0 * 1024.0);
        }
    }

    bool try_mount_disk(UsbDiskInfo& d) {
        if (is_mounted(d.mount_point)) {
            d.mounted = true;
            read_disk_sysfs(d);
            return true;
        }

        std::string part = find_disk_partition(d.dev_name);
        if (part.empty()) {
            d.mounted = false;
            return false;
        }

        d.part_name = part;
        if (mkdir(d.mount_point.c_str(), 0755) < 0 && errno != EEXIST) {}

        // Lazy unmount any stale broken mount point
        std::string umnt = "umount -l " + d.mount_point + " 2>/dev/null";
        if (system(umnt.c_str()) < 0) {}

        std::string mnt_cmd;
        if (d.fs_type == "exfat-fuse") {
            mnt_cmd = "mount.exfat-fuse " + d.part_name + " " + d.mount_point + " 2>/dev/null || "
                      "mount -t exfat " + d.part_name + " " + d.mount_point + " 2>/dev/null || "
                      "mount " + d.part_name + " " + d.mount_point + " 2>/dev/null";
        } else {
            mnt_cmd = "mount -t ntfs-3g " + d.part_name + " " + d.mount_point + " 2>/dev/null || "
                      "mount " + d.part_name + " " + d.mount_point + " 2>/dev/null";
        }

        if (system(mnt_cmd.c_str()) == 0) {
            d.mounted = true;
            read_disk_sysfs(d);
            return true;
        }

        d.mounted = false;
        return false;
    }

    void init_disks() {
        disks.clear();
        disks.push_back(std::make_unique<UsbDiskInfo>("sda", "/dev/sda2", "/mnt/rk_usb_test/sda", "USB 1 (sda)", "ntfs-3g"));
        disks.push_back(std::make_unique<UsbDiskInfo>("sdb", "/dev/sdb1", "/mnt/rk_usb_test/sdb", "USB 2 (sdb)", "ntfs-3g"));
        disks.push_back(std::make_unique<UsbDiskInfo>("sdc", "/dev/sdc2", "/mnt/rk_usb_test/sdc", "USB 3 (sdc)", "ntfs-3g"));
        disks.push_back(std::make_unique<UsbDiskInfo>("sdd", "/dev/sdd1", "/mnt/rk_usb_test/sdd", "USB 4 (sdd)", "exfat-fuse"));
        check_and_mount();
    }

    void check_and_mount() {
        if (system("mkdir -p /mnt/rk_usb_test/{sda,sdb,sdc,sdd} 2>/dev/null") < 0) {}
        for (auto& d : disks) {
            try_mount_disk(*d);
        }
    }

    void check_and_update_disks() {
        for (auto& d : disks) {
            if (is_mounted(d->mount_point)) {
                d->mounted = true;
                continue;
            }

            std::string part = find_disk_partition(d->dev_name);
            if (part.empty()) {
                if (d->mounted) {
                    d->mounted = false;
                    std::string umnt = "umount -l " + d->mount_point + " 2>/dev/null";
                    if (system(umnt.c_str()) < 0) {}
                }
            } else {
                try_mount_disk(*d);
            }
        }
    }

    void start() {
        if (active.load()) return;
        check_and_mount();
        active.store(true);
        workers.clear();
        for (size_t i = 0; i < disks.size(); ++i) {
            workers.emplace_back(&UsbStorageTester::worker_loop, this, i);
        }
    }

    void stop() {
        if (!active.load()) return;
        active.store(false);
        for (auto& t : workers) {
            if (t.joinable()) t.join();
        }
        workers.clear();
        for (auto& d : disks) {
            d->write_speed_mb.store(0.0);
            d->read_speed_mb.store(0.0);
        }
    }

    void reset() {
        for (auto& d : disks) {
            d->write_bytes.store(0);
            d->read_bytes.store(0);
            d->write_ops.store(0);
            d->read_ops.store(0);
            d->errors.store(0);
            d->disconnects.store(0);
            d->recovers.store(0);
            d->write_speed_mb.store(0.0);
            d->read_speed_mb.store(0.0);
        }
    }

    uint64_t get_total_write_mb() const {
        uint64_t s = 0;
        for (const auto& d : disks) s += (d->write_bytes.load() / (1024 * 1024));
        return s;
    }

    uint64_t get_total_read_mb() const {
        uint64_t s = 0;
        for (const auto& d : disks) s += (d->read_bytes.load() / (1024 * 1024));
        return s;
    }

    uint64_t get_total_errors() const {
        uint64_t s = 0;
        for (const auto& d : disks) s += d->errors.load();
        return s;
    }

    double get_total_write_speed() const {
        double s = 0.0;
        for (const auto& d : disks) s += d->write_speed_mb.load();
        return s;
    }

    double get_total_read_speed() const {
        double s = 0.0;
        for (const auto& d : disks) s += d->read_speed_mb.load();
        return s;
    }

private:
    void worker_loop(size_t idx) {
        auto& d = *disks[idx];
        std::string test_file = d.mount_point + "/.rk_test_io_" + d.dev_name + ".dat";
        const size_t BLK_SIZE = 1 * 1024 * 1024; // 1MB test block
        std::vector<uint8_t> w_buf(BLK_SIZE);
        std::vector<uint8_t> r_buf(BLK_SIZE);

        uint32_t seed = (uint32_t)(idx * 1000 + 1);

        auto last_calc = std::chrono::steady_clock::now();
        uint64_t w_since_calc = 0;
        uint64_t r_since_calc = 0;

        while (active.load()) {
            if (!d.mounted) {
                d.write_speed_mb.store(0.0);
                d.read_speed_mb.store(0.0);

                if (try_mount_disk(d)) {
                    d.recovers.fetch_add(1);
                } else {
                    usleep(500000);
                    continue;
                }
            }

            for (size_t i = 0; i < BLK_SIZE; i += 4) {
                uint32_t val = seed + (uint32_t)i;
                std::memcpy(&w_buf[i], &val, 4);
            }
            seed += 17;

            // 1. Write phase
            int fd = open(test_file.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0666);
            if (fd < 0) {
                if (active.load()) {
                    d.errors.fetch_add(1);
                    std::string part = find_disk_partition(d.dev_name);
                    if (part.empty()) {
                        d.mounted = false;
                        d.disconnects.fetch_add(1);
                        std::string umnt = "umount -l " + d.mount_point + " 2>/dev/null";
                        if (system(umnt.c_str()) < 0) {}
                    }
                }
                usleep(300000);
                continue;
            }

            size_t written = 0;
            bool write_err = false;
            while (written < BLK_SIZE && active.load()) {
                ssize_t w = write(fd, w_buf.data() + written, BLK_SIZE - written);
                if (w <= 0) {
                    write_err = true;
                    break;
                }
                written += w;
            }
            if (written == BLK_SIZE && !write_err) {
                if (fdatasync(fd) < 0) {
                    write_err = true;
                }
            }
            close(fd);

            if (write_err || written < BLK_SIZE) {
                if (active.load()) {
                    d.errors.fetch_add(1);
                    std::string part = find_disk_partition(d.dev_name);
                    if (part.empty()) {
                        d.mounted = false;
                        d.disconnects.fetch_add(1);
                        std::string umnt = "umount -l " + d.mount_point + " 2>/dev/null";
                        if (system(umnt.c_str()) < 0) {}
                    }
                }
                unlink(test_file.c_str());
                usleep(300000);
                continue;
            }

            d.write_bytes.fetch_add(written);
            d.write_ops.fetch_add(1);
            w_since_calc += written;

            if (!active.load()) break;

            // 2. Read phase
            int rfd = open(test_file.c_str(), O_RDONLY);
            if (rfd < 0) {
                if (active.load()) {
                    d.errors.fetch_add(1);
                    std::string part = find_disk_partition(d.dev_name);
                    if (part.empty()) {
                        d.mounted = false;
                        d.disconnects.fetch_add(1);
                        std::string umnt = "umount -l " + d.mount_point + " 2>/dev/null";
                        if (system(umnt.c_str()) < 0) {}
                    }
                }
                unlink(test_file.c_str());
                usleep(300000);
                continue;
            }

            size_t read_bytes = 0;
            bool read_err = false;
            while (read_bytes < BLK_SIZE && active.load()) {
                ssize_t r = read(rfd, r_buf.data() + read_bytes, BLK_SIZE - read_bytes);
                if (r <= 0) {
                    read_err = true;
                    break;
                }
                read_bytes += r;
            }
            close(rfd);

            if (read_err || read_bytes < BLK_SIZE) {
                if (active.load()) {
                    d.errors.fetch_add(1);
                    std::string part = find_disk_partition(d.dev_name);
                    if (part.empty()) {
                        d.mounted = false;
                        d.disconnects.fetch_add(1);
                        std::string umnt = "umount -l " + d.mount_point + " 2>/dev/null";
                        if (system(umnt.c_str()) < 0) {}
                    }
                }
                unlink(test_file.c_str());
                usleep(300000);
                continue;
            }

            d.read_bytes.fetch_add(read_bytes);
            d.read_ops.fetch_add(1);
            r_since_calc += read_bytes;

            // 3. Verify data integrity
            if (std::memcmp(w_buf.data(), r_buf.data(), BLK_SIZE) != 0) {
                if (active.load()) d.errors.fetch_add(1);
            }

            unlink(test_file.c_str());

            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calc).count() / 1000.0;
            if (dt >= 0.5) {
                last_calc = now;
                double w_mb_s = (w_since_calc / (1024.0 * 1024.0)) / dt;
                double r_mb_s = (r_since_calc / (1024.0 * 1024.0)) / dt;
                d.write_speed_mb.store(w_mb_s);
                d.read_speed_mb.store(r_mb_s);
                w_since_calc = 0;
                r_since_calc = 0;
            }

            usleep(20000);
        }

        unlink(test_file.c_str());
        d.write_speed_mb.store(0.0);
        d.read_speed_mb.store(0.0);
    }
};

class RkTesterEngine {
public:
    int num_cores{8};
    std::vector<CpuTime> prev_cores;
    CpuTime prev_total;
    std::map<std::string, std::string> thermal_zones;
    std::vector<NetPortInfo> net_ports;

    // 压测状态
    std::atomic<bool> cpu_stress{false};
    std::atomic<bool> gpu_stress{false};
    std::atomic<bool> npu_stress{false};
    int target_cpu_cores{8};
    bool governor_perf{false};

    std::vector<std::thread> cpu_threads;
    pid_t gpu_pid{0};
    std::vector<pid_t> npu_pids;

    // EMC 丢包测试状态
    std::atomic<bool> emc_active{false};
    std::string emc_target_ip{"192.168.32.100"};
    int emc_interval_ms{100};  // 100ms 高频发包
    int emc_packet_size{1000}; // 1000 字节大包

    std::string emc_latest_event{"EMC test on STANDBY. Press [5] to start monitoring."};
    int emc_latest_event_color{1};

    std::ofstream emc_log_file;
    std::string emc_log_path;

    // 串口单向测试
    SerialChannel rs232{"RS232 (ttyS0->S1)", "/dev/ttyS0", "/dev/ttyS1"};
    SerialChannel rs485{"RS485 (ttyS2->S3)", "/dev/ttyS2", "/dev/ttyS3"};

    // 4路 USB 存储读写压测
    UsbStorageTester usb_tester;

    // CAN0 <-> CAN1 互发测试
    CanBusTester can_tester;

    void toggle_rs232() {
        if (rs232.active.load()) {
            rs232.stop();
        } else {
            rs232.start();
        }
    }

    void toggle_rs485() {
        if (rs485.active.load()) {
            rs485.stop();
        } else {
            rs485.start();
        }
    }

    void toggle_usb_stress() {
        if (usb_tester.active.load()) {
            usb_tester.stop();
        } else {
            usb_tester.start();
        }
    }

    void toggle_can() {
        if (can_tester.active.load()) {
            can_tester.stop();
        } else {
            can_tester.start();
        }
    }

    uint64_t get_total_sent() const {
        uint64_t s = 0;
        for (const auto& p : net_ports) s += p.sent;
        return s;
    }
    uint64_t get_total_recv() const {
        uint64_t s = 0;
        for (const auto& p : net_ports) s += p.recv;
        return s;
    }
    uint64_t get_total_lost() const {
        uint64_t s = 0;
        for (const auto& p : net_ports) s += p.lost;
        return s;
    }
    uint64_t get_total_flaps() const {
        uint64_t s = 0;
        for (const auto& p : net_ports) s += p.flaps;
        return s;
    }
    void reset_emc_stats() {
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
        rs232.reset();
        rs485.reset();
        can_tester.reset();
        usb_tester.reset();
    }

    RkTesterEngine() {
        num_cores = sysconf(_SC_NPROCESSORS_ONLN);
        if (num_cores <= 0) num_cores = 8;
        target_cpu_cores = num_cores;
        prev_cores.resize(num_cores);

        init_thermals();
        init_net_ports();
        setup_default_network();
        update_net_status();
        init_emc();
    }

    ~RkTesterEngine() {
        stop_all_stress();
        for (auto& port : net_ports) {
            if (port.raw_sock >= 0) close(port.raw_sock);
        }
        if (emc_log_file.is_open()) emc_log_file.close();
    }

    void init_thermals() {
        for (int i = 0; i < 16; ++i) {
            std::string t_path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/type";
            std::string val_path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/temp";
            std::ifstream t_file(t_path);
            if (!t_file.is_open()) break;
            std::string name;
            std::getline(t_file, name);
            if (!name.empty() && name.back() == '\r') name.pop_back();
            size_t pos = name.find("-thermal");
            if (pos != std::string::npos) name = name.substr(0, pos);
            thermal_zones[name] = val_path;
        }
    }

    void init_net_ports() {
        net_ports = {
            {"eth0", "Port 1 (USB/eth0)",  "192.168.32.101", "", false, -1, -1, 0, 0, 0, 0, 0, 0.0, 999999.0, 0.0, 0.0, {}, true},
            {"eth1", "Port 2 (PCIe/eth1)", "192.168.32.102", "", false, -1, -1, 0, 0, 0, 0, 0, 0.0, 999999.0, 0.0, 0.0, {}, true},
            {"eth2", "Port 3 (GMAC0/eth2)","192.168.32.103", "", false, -1, -1, 0, 0, 0, 0, 0, 0.0, 999999.0, 0.0, 0.0, {}, true},
            {"eth3", "Port 4 (GMAC1/eth3)","192.168.32.104", "", false, -1, -1, 0, 0, 0, 0, 0, 0.0, 999999.0, 0.0, 0.0, {}, true}
        };
    }

    void setup_default_network() {
        if (system("sysctl -q -w net.ipv4.conf.all.arp_ignore=1 2>/dev/null") < 0) {}
        if (system("sysctl -q -w net.ipv4.conf.all.arp_announce=2 2>/dev/null") < 0) {}

        struct Target {
            const char* iface;
            const char* ip;
            int metric;
        } targets[] = {
            {"eth0", "192.168.32.101", 100},
            {"eth1", "192.168.32.102", 301},
            {"eth2", "192.168.32.103", 102},
            {"eth3", "192.168.32.104", 101}
        };

        for (const auto& t : targets) {
            std::string up_cmd = "ip link set dev " + std::string(t.iface) + " up 2>/dev/null";
            if (system(up_cmd.c_str()) < 0) {}

            std::string chk_cmd = "ip addr show dev " + std::string(t.iface) + " | grep -q 'inet " + t.ip + "/'";
            if (system(chk_cmd.c_str()) != 0) {
                std::string add_cmd = "ip addr add " + std::string(t.ip) + "/24 dev " + t.iface + " 2>/dev/null";
                if (system(add_cmd.c_str()) < 0) {}
            }

            std::string del_rt = "ip route del 192.168.32.0/24 dev " + std::string(t.iface) + " metric 0 2>/dev/null";
            if (system(del_rt.c_str()) < 0) {}
            std::string add_rt = "ip route add 192.168.32.0/24 dev " + std::string(t.iface) + " metric " + std::to_string(t.metric) + " 2>/dev/null";
            if (system(add_rt.c_str()) < 0) {}
        }
    }

    void set_target_ip(const std::string& new_ip) {
        emc_target_ip = new_ip;
        reset_emc_stats();

        // Check if target IP belongs to a different /24 subnet
        size_t last_dot = new_ip.rfind('.');
        if (last_dot != std::string::npos) {
            std::string subnet = new_ip.substr(0, last_dot);
            if (subnet != "192.168.32") {
                struct TargetMetric {
                    int metric;
                } metrics[] = {{100}, {301}, {102}, {101}};
                for (size_t i = 0; i < net_ports.size(); ++i) {
                    std::string port_ip = subnet + "." + std::to_string(101 + i);
                    net_ports[i].target_ip = port_ip;

                    std::string chk = "ip addr show dev " + net_ports[i].name + " | grep -q 'inet " + port_ip + "/'";
                    if (system(chk.c_str()) != 0) {
                        std::string cmd = "ip addr add " + port_ip + "/24 dev " + net_ports[i].name + " 2>/dev/null";
                        if (system(cmd.c_str()) < 0) {}
                    }
                    std::string rt_cmd = "ip route add " + subnet + ".0/24 dev " + net_ports[i].name + " metric " + std::to_string(metrics[i].metric) + " 2>/dev/null";
                    if (system(rt_cmd.c_str()) < 0) {}
                }
                update_net_status();
                log_emc_event("INFO", "Auto-configured 4-LAN IPs for subnet " + subnet + ".0/24", 4);
            } else {
                for (size_t i = 0; i < net_ports.size(); ++i) {
                    net_ports[i].target_ip = "192.168.32." + std::to_string(101 + i);
                }
                update_net_status();
            }
        }
    }

    void init_emc() {
        uint16_t base_id = (uint16_t)(getpid() & 0x3FFF);
        for (size_t i = 0; i < net_ports.size(); ++i) {
            auto& port = net_ports[i];
            port.ping_id = base_id + (uint16_t)(i * 1000 + 100);
            port.raw_sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
            if (port.raw_sock >= 0) {
                struct ifreq ifr;
                std::memset(&ifr, 0, sizeof(ifr));
                std::strncpy(ifr.ifr_name, port.name.c_str(), IFNAMSIZ - 1);
                setsockopt(port.raw_sock, SOL_SOCKET, SO_BINDTODEVICE, (void *)&ifr, sizeof(ifr));

                int flags = fcntl(port.raw_sock, F_GETFL, 0);
                fcntl(port.raw_sock, F_SETFL, flags | O_NONBLOCK);
            }
        }

        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "/tmp/emc_test_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << ".log";
        emc_log_path = ss.str();
        emc_log_file.open(emc_log_path, std::ios::out | std::ios::app);
    }

    void log_emc_event(const std::string& level, const std::string& msg, int color) {
        std::string ts = get_time_str();
        emc_latest_event = "[" + ts + "] " + msg;
        emc_latest_event_color = color;
        if (emc_log_file.is_open()) {
            emc_log_file << "[" << ts << "] [" << level << "] " << msg << std::endl;
            emc_log_file.flush();
        }
    }

    // EMC 发包 (逐个独立网口发送)
    void emc_tick_send() {
        if (!emc_active.load()) return;

        for (auto& port : net_ports) {
            if (!port.enabled) continue;

            if (!port.carrier || port.raw_sock < 0) {
                // Link is down / cable unplugged during active test:
                // Count packet attempt and record as LOST!
                port.sent++;
                port.lost++;
                continue;
            }

            std::vector<char> send_buf(emc_packet_size, 0);
            struct icmphdr *icmp = (struct icmphdr *)send_buf.data();
            icmp->type = ICMP_ECHO;
            icmp->code = 0;
            icmp->un.echo.id = htons(port.ping_id);
            uint16_t seq = (uint16_t)(port.sent & 0xFFFF);
            icmp->un.echo.sequence = htons(seq);

            uint64_t now_us = get_time_us();
            char *payload = send_buf.data() + sizeof(struct icmphdr);
            std::memcpy(payload, &now_us, sizeof(now_us));

            icmp->checksum = 0;
            icmp->checksum = in_cksum((uint16_t *)send_buf.data(), emc_packet_size);

            struct sockaddr_in dest;
            std::memset(&dest, 0, sizeof(dest));
            dest.sin_family = AF_INET;
            inet_pton(AF_INET, emc_target_ip.c_str(), &dest.sin_addr);

            ssize_t ret = sendto(port.raw_sock, send_buf.data(), emc_packet_size, 0,
                                 (struct sockaddr *)&dest, sizeof(dest));
            if (ret > 0) {
                port.sent++;
                SentPacket sp;
                sp.send_time_us = now_us;
                sp.replied = false;
                sp.marked_lost = false;
                port.sent_map[seq] = sp;
            } else {
                // Hardware transmission failed (e.g. cable disconnected just as packet was sending)
                port.sent++;
                port.lost++;
            }
        }
    }

    // EMC 收包 (逐个独立网口接收)
    void emc_tick_recv() {
        for (auto& port : net_ports) {
            if (port.raw_sock < 0) continue;

            char recv_buf[2048];
            struct sockaddr_in from;
            socklen_t fromlen = sizeof(from);

            while (true) {
                ssize_t len = recvfrom(port.raw_sock, recv_buf, sizeof(recv_buf), 0,
                                       (struct sockaddr *)&from, &fromlen);
                if (len <= 0) break;

                struct iphdr *ip = (struct iphdr *)recv_buf;
                int hlen = ip->ihl << 2;
                if (len < hlen + (int)sizeof(struct icmphdr)) continue;

                struct icmphdr *icmp = (struct icmphdr *)(recv_buf + hlen);
                if (icmp->type == ICMP_ECHOREPLY && ntohs(icmp->un.echo.id) == port.ping_id) {
                    uint16_t seq = ntohs(icmp->un.echo.sequence);
                    uint64_t now_us = get_time_us();

                    if (port.sent_map.count(seq)) {
                        auto& sp = port.sent_map[seq];
                        if (!sp.replied) {
                            sp.replied = true;
                            port.recv++;

                            double rtt_ms = (now_us - sp.send_time_us) / 1000.0;
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

    // EMC 超时检测 (每个独立网口分别判定丢包)
    void emc_tick_timeout() {
        uint64_t now_us = get_time_us();
        uint64_t timeout_us = 1000000; // 1秒超时

        for (auto& port : net_ports) {
            auto it = port.sent_map.begin();
            while (it != port.sent_map.end()) {
                if (!it->second.replied && !it->second.marked_lost) {
                    if (now_us - it->second.send_time_us > timeout_us) {
                        it->second.marked_lost = true;
                        port.lost++;
                        log_emc_event("DROP", "WARNING: " + port.ch_name + " Seq #" +
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

    void update_net_status() {
        struct ifaddrs* ifaddr = nullptr;
        std::map<std::string, std::vector<std::string>> iface_ips;
        if (getifaddrs(&ifaddr) != -1) {
            for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
                if (!ifa->ifa_addr) continue;
                if (ifa->ifa_addr->sa_family == AF_INET) {
                    char host[INET_ADDRSTRLEN];
                    void* addr = &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;
                    inet_ntop(AF_INET, addr, host, sizeof(host));
                    iface_ips[ifa->ifa_name].push_back(host);
                }
            }
            freeifaddrs(ifaddr);
        }

        for (auto& port : net_ports) {
            std::string iface = port.name;

            if (iface_ips.count(iface) && !iface_ips[iface].empty()) {
                port.current_ip = iface_ips[iface][0];
                for (const auto& ip : iface_ips[iface]) {
                    if (ip == port.target_ip) {
                        port.current_ip = port.target_ip;
                        break;
                    }
                }
            } else {
                port.current_ip = "No IP";
            }

            std::ifstream fc("/sys/class/net/" + iface + "/carrier");
            int c = 0;
            bool carrier = false;
            if (fc.is_open() && (fc >> c)) carrier = (c == 1);

            if (carrier != port.carrier) {
                if (port.carrier && !carrier) {
                    port.flaps++;
                    log_emc_event("FLAP", "CRITICAL: " + port.ch_name + " Link Down (cable unplugged)!", 3);
                } else if (!port.carrier && carrier) {
                    std::string spd_str = port.speed > 0 ? std::to_string(port.speed) : "1000";
                    log_emc_event("INFO", "INFO: " + port.ch_name + " Link Up (" + spd_str + "M)! Auto-resuming ping.", 1);
                    // Ensure IP address is configured on this port if missing
                    if (port.current_ip == "No IP" || port.current_ip.empty()) {
                        std::string cmd = "ip addr add " + port.target_ip + "/24 dev " + port.name + " 2>/dev/null; ip link set " + port.name + " up";
                        if (system(cmd.c_str()) < 0) {}
                    }
                    // Ensure raw socket is healthy
                    if (port.raw_sock < 0) {
                        port.raw_sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
                        if (port.raw_sock >= 0) {
                            struct ifreq ifr;
                            std::memset(&ifr, 0, sizeof(ifr));
                            std::strncpy(ifr.ifr_name, port.name.c_str(), IFNAMSIZ - 1);
                            setsockopt(port.raw_sock, SOL_SOCKET, SO_BINDTODEVICE, (void *)&ifr, sizeof(ifr));
                            int flags = fcntl(port.raw_sock, F_GETFL, 0);
                            fcntl(port.raw_sock, F_SETFL, flags | O_NONBLOCK);
                        }
                    }
                }
                port.carrier = carrier;
            }

            std::ifstream fs("/sys/class/net/" + iface + "/speed");
            int sp = -1;
            if (fs.is_open() && (fs >> sp)) port.speed = sp;
            else port.speed = -1;
        }
    }

    // CPU 压测
    void toggle_cpu_stress() {
        if (cpu_stress.load()) {
            cpu_stress.store(false);
            for (auto& t : cpu_threads) {
                if (t.joinable()) t.join();
            }
            cpu_threads.clear();
        } else {
            cpu_stress.store(true);
            for (int i = 0; i < target_cpu_cores; ++i) {
                cpu_threads.emplace_back([this, i]() {
                    cpu_set_t cpuset;
                    CPU_ZERO(&cpuset);
                    CPU_SET(i % num_cores, &cpuset);
                    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

                    volatile double x = 1.00001;
                    while (this->cpu_stress.load(std::memory_order_relaxed)) {
                        for (int k = 0; k < 50000; ++k) {
                            x = x * 1.000002 + 0.000005;
                            if (x > 1000.0) x = 1.00001;
                        }
                    }
                });
            }
        }
    }

    // GPU 压测
    void toggle_gpu_stress() {
        if (gpu_stress.load()) {
            if (gpu_pid > 0) {
                kill(-gpu_pid, SIGTERM);
                waitpid(gpu_pid, nullptr, WNOHANG);
                gpu_pid = 0;
            }
            if (system("killall -9 glmark2-es2-drm 2>/dev/null") < 0) {}
            gpu_stress.store(false);
        } else {
            pid_t pid = fork();
            if (pid == 0) {
                setsid();
                int devnull = open("/dev/null", O_WRONLY);
                dup2(devnull, STDOUT_FILENO);
                dup2(devnull, STDERR_FILENO);
                close(devnull);
                execlp("glmark2-es2-drm", "glmark2-es2-drm", "--off-screen", "--run-forever", nullptr);
                _exit(1);
            } else if (pid > 0) {
                gpu_pid = pid;
                gpu_stress.store(true);
            }
        }
    }

    // NPU 压测
    void toggle_npu_stress() {
        if (npu_stress.load()) {
            for (pid_t p : npu_pids) {
                if (p > 0) {
                    kill(-p, SIGKILL);
                    waitpid(p, nullptr, WNOHANG);
                }
            }
            npu_pids.clear();
            if (system("killall -9 rknn_common_test 2>/dev/null") < 0) {}
            npu_stress.store(false);
        } else {
            npu_pids.clear();
            for (int i = 0; i < 2; ++i) {
                pid_t pid = fork();
                if (pid == 0) {
                    setsid();
                    int devnull = open("/dev/null", O_WRONLY);
                    dup2(devnull, STDOUT_FILENO);
                    dup2(devnull, STDERR_FILENO);
                    close(devnull);
                    while (true) {
                        pid_t child = fork();
                        if (child == 0) {
                            execl(RKNN_TEST_BIN.c_str(), RKNN_TEST_BIN.c_str(),
                                  RKNN_MODEL.c_str(), RKNN_IMG.c_str(), "500", nullptr);
                            _exit(1);
                        } else if (child > 0) {
                            waitpid(child, nullptr, 0);
                        }
                    }
                } else if (pid > 0) {
                    npu_pids.push_back(pid);
                }
            }
            npu_stress.store(true);
        }
    }

    void toggle_all_stress() {
        bool all_on = cpu_stress.load() && gpu_stress.load() && npu_stress.load();
        if (all_on) {
            stop_all_stress();
        } else {
            if (!cpu_stress.load()) toggle_cpu_stress();
            if (!gpu_stress.load()) toggle_gpu_stress();
            if (!npu_stress.load()) toggle_npu_stress();
        }
    }

    void stop_all_stress() {
        if (cpu_stress.load()) toggle_cpu_stress();
        if (gpu_stress.load()) toggle_gpu_stress();
        if (npu_stress.load()) toggle_npu_stress();
        rs232.stop();
        rs485.stop();
        can_tester.stop();
        usb_tester.stop();
    }

    void toggle_governor() {
        governor_perf = !governor_perf;
        const char* gov = governor_perf ? "performance" : "schedutil";
        for (int i = 0; i < num_cores; ++i) {
            std::string p = "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_governor";
            std::ofstream f(p);
            if (f.is_open()) f << gov;
        }
        std::ofstream fg(DEVFREQ_GPU + "/governor");
        if (fg.is_open()) fg << gov;
        std::ofstream fn(DEVFREQ_NPU + "/governor");
        if (fn.is_open()) fn << gov;
    }

    std::pair<double, std::vector<double>> get_cpu_loads() {
        std::ifstream stat_file("/proc/stat");
        if (!stat_file.is_open()) return {0.0, std::vector<double>(num_cores, 0.0)};

        std::string line;
        CpuTime cur_total;
        std::vector<CpuTime> cur_cores(num_cores);

        while (std::getline(stat_file, line)) {
            if (line.rfind("cpu", 0) != 0) break;
            std::istringstream ss(line);
            std::string cpu_lbl;
            unsigned long long u, n, s, id, io, ir, so;
            ss >> cpu_lbl >> u >> n >> s >> id >> io >> ir >> so;
            unsigned long long tot = u + n + s + id + io + ir + so;
            unsigned long long idl = id + io;

            if (cpu_lbl == "cpu") {
                cur_total.total = tot;
                cur_total.idle = idl;
            } else if (cpu_lbl.size() > 3) {
                int c_idx = std::stoi(cpu_lbl.substr(3));
                if (c_idx >= 0 && c_idx < num_cores) {
                    cur_cores[c_idx].total = tot;
                    cur_cores[c_idx].idle = idl;
                }
            }
        }

        double total_load = 0.0;
        if (prev_total.total > 0 && cur_total.total > prev_total.total) {
            unsigned long long d_tot = cur_total.total - prev_total.total;
            unsigned long long d_idl = cur_total.idle - prev_total.idle;
            total_load = (d_tot > d_idl) ? (double)(d_tot - d_idl) * 100.0 / d_tot : 0.0;
        }
        prev_total = cur_total;

        std::vector<double> core_loads(num_cores, 0.0);
        for (int i = 0; i < num_cores; ++i) {
            if (prev_cores[i].total > 0 && cur_cores[i].total > prev_cores[i].total) {
                unsigned long long d_tot = cur_cores[i].total - prev_cores[i].total;
                unsigned long long d_idl = cur_cores[i].idle - prev_cores[i].idle;
                core_loads[i] = (d_tot > d_idl) ? (double)(d_tot - d_idl) * 100.0 / d_tot : 0.0;
            }
            prev_cores[i] = cur_cores[i];
        }

        return {total_load, core_loads};
    }

    std::vector<int> get_cpu_freqs() {
        std::vector<int> freqs(num_cores, 0);
        for (int i = 0; i < num_cores; ++i) {
            std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq";
            std::ifstream f(path);
            if (f.is_open()) {
                int khz = 0;
                if (f >> khz) freqs[i] = khz / 1000;
            }
        }
        return freqs;
    }

    std::pair<double, int> get_gpu_stats() {
        double load = 0.0;
        int freq_mhz = 0;
        std::ifstream f(DEVFREQ_GPU + "/load");
        if (f.is_open()) {
            std::string s;
            f >> s;
            size_t at_pos = s.find('@');
            if (at_pos != std::string::npos) {
                try { load = std::stod(s.substr(0, at_pos)); } catch (...) {}
                size_t hz_pos = s.find("Hz", at_pos);
                if (hz_pos != std::string::npos) {
                    try {
                        std::string f_str = s.substr(at_pos + 1, hz_pos - at_pos - 1);
                        freq_mhz = std::stoll(f_str) / 1000000;
                    } catch (...) {}
                }
            }
        }
        if (freq_mhz == 0) {
            std::ifstream cf(DEVFREQ_GPU + "/cur_freq");
            if (cf.is_open()) {
                long long hz = 0;
                if (cf >> hz) freq_mhz = hz / 1000000;
            }
        }
        return {load, freq_mhz};
    }

    std::pair<std::vector<double>, int> get_npu_stats() {
        std::vector<double> loads = {0.0, 0.0};
        int freq_mhz = 0;
        std::ifstream f(DEBUG_RKNPU_LOAD);
        if (f.is_open()) {
            std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
            std::regex re("Core(\\d+):\\s*(\\d+)%");
            auto begin = std::sregex_iterator(content.begin(), content.end(), re);
            auto end = std::sregex_iterator();
            for (auto it = begin; it != end; ++it) {
                int core = std::stoi((*it)[1].str());
                double val = std::stod((*it)[2].str());
                if (core >= 0 && core < (int)loads.size()) {
                    loads[core] = val;
                }
            }
        }
        std::ifstream cf(DEVFREQ_NPU + "/cur_freq");
        if (cf.is_open()) {
            long long hz = 0;
            if (cf >> hz) freq_mhz = hz / 1000000;
        }
        return {loads, freq_mhz};
    }

    std::map<std::string, double> get_thermals() {
        std::map<std::string, double> res;
        for (const auto& kv : thermal_zones) {
            std::ifstream f(kv.second);
            if (f.is_open()) {
                int raw_temp = 0;
                if (f >> raw_temp) res[kv.first] = raw_temp / 1000.0;
            }
        }
        return res;
    }

    std::tuple<double, double, double> get_mem_stats() {
        std::ifstream f("/proc/meminfo");
        long long total = 0, avail = 0;
        std::string line;
        while (std::getline(f, line)) {
            if (line.rfind("MemTotal:", 0) == 0) {
                std::istringstream ss(line.substr(9));
                ss >> total;
            } else if (line.rfind("MemAvailable:", 0) == 0) {
                std::istringstream ss(line.substr(13));
                ss >> avail;
            }
        }
        double tot_gb = total / (1024.0 * 1024.0);
        double used_gb = (total - avail) / (1024.0 * 1024.0);
        double pct = (total > 0) ? ((total - avail) * 100.0 / total) : 0.0;
        return {used_gb, tot_gb, pct};
    }

    std::string get_uptime() {
        std::ifstream f("/proc/uptime");
        double up = 0;
        if (f >> up) {
            long long s = (long long)up;
            int hrs = s / 3600;
            int mins = (s % 3600) / 60;
            int secs = s % 60;
            char buf[32];
            snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hrs, mins, secs);
            return buf;
        }
        return "N/A";
    }
};

static RkTesterEngine* g_engine = nullptr;

void print_summary_report(RkTesterEngine* engine) {
    if (!engine) return;
    std::cout << "\n====================================================================\n";
    std::cout << "        RK3576 Hardware Test Summary (4-LAN + Serial + USB)\n";
    std::cout << "====================================================================\n";
    std::cout << " Target Host IP     : " << engine->emc_target_ip << "\n";

    uint64_t tot_sent = engine->get_total_sent();
    uint64_t tot_recv = engine->get_total_recv();
    uint64_t tot_lost = engine->get_total_lost();
    uint64_t tot_flaps = engine->get_total_flaps();
    double tot_loss_pct = (tot_sent > 0) ? (tot_lost * 100.0 / tot_sent) : 0.0;

    if (tot_sent == 0) {
        std::cout << " Overall Immunity   : NOT TESTED (4-LAN ping test was not started, 0 packets sent)\n";
    } else if (tot_flaps == 0 && tot_lost == 0) {
        std::cout << " Overall Immunity   : CLASS A (PASS: 0 Packet Loss, 0 Link Flaps)\n";
    } else if (tot_flaps == 0 && tot_loss_pct < 1.0) {
        std::cout << " Overall Immunity   : CLASS B (MARGINAL: Transient loss < 1%, Self-recovered)\n";
    } else {
        std::cout << " Overall Immunity   : CLASS C (FAIL: Link Flap or Excessive Packet Loss)\n";
    }
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " 4-LAN Port Ping & Loss Breakdown:\n";
    for (size_t i = 0; i < engine->net_ports.size(); ++i) {
        const auto& p = engine->net_ports[i];
        double p_loss = (p.sent > 0) ? (p.lost * 100.0 / p.sent) : 0.0;
        char buf[128];
        snprintf(buf, sizeof(buf), " * %-19s (IP: %-15s): Sent: %-5llu Recv: %-5llu Lost: %-3llu (%4.1f%%) Flaps: %-2llu",
                 p.ch_name.c_str(), p.current_ip.c_str(),
                 (unsigned long long)p.sent, (unsigned long long)p.recv,
                 (unsigned long long)p.lost, p_loss, (unsigned long long)p.flaps);
        std::cout << buf;
        if (!p.enabled) {
            std::cout << " [DISABLED]\n";
        } else if (!p.carrier && p.sent == 0) {
            std::cout << " [NOT CONNECTED]\n";
        } else if (p.flaps == 0 && p.lost == 0) {
            std::cout << " [PASS]\n";
        } else {
            std::cout << " [FAIL/GLITCH]\n";
        }
    }
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " Serial Port Unidirectional Transmission Summary:\n";
    auto print_serial_summary = [](const std::string& title, const SerialChannel& sc) {
        uint64_t tx_k = sc.tx_bytes.load() / 1024;
        uint64_t rx_k = sc.rx_bytes.load() / 1024;
        uint64_t tx_f = sc.tx_frames.load();
        uint64_t rx_f = sc.rx_frames.load();
        uint64_t err_f = sc.err_frames.load();
        double loss_pct = (tx_f > 0) ? (err_f * 100.0 / tx_f) : 0.0;
        char buf[256];
        snprintf(buf, sizeof(buf), " * %-22s: TX: %llu KB (%llu pkts), RX: %llu KB (%llu pkts), Err: %llu (%4.1f%%)",
                 title.c_str(),
                 (unsigned long long)tx_k, (unsigned long long)tx_f,
                 (unsigned long long)rx_k, (unsigned long long)rx_f,
                 (unsigned long long)err_f, loss_pct);
        std::cout << buf;
        if (tx_f == 0) {
            std::cout << " [NOT TESTED]\n";
        } else if (err_f == 0) {
            std::cout << " [PASS]\n";
        } else {
            std::cout << " [FAIL/ERRORS]\n";
        }
    };
    print_serial_summary("RS232 (ttyS0 -> ttyS1)", engine->rs232);
    print_serial_summary("RS485 (ttyS2 -> ttyS3)", engine->rs485);
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " CAN Bus (can0 / can1) Transmission Summary:\n";
    std::string mode_str = engine->can_tester.get_mode_name();
    uint64_t can_tx = engine->can_tester.tx_frames.load();
    uint64_t can_rx = engine->can_tester.rx_frames.load();
    uint64_t can_err = engine->can_tester.err_frames.load();
    double can_loss_pct = (can_tx > 0) ? (can_err * 100.0 / can_tx) : 0.0;
    std::cout << " * Mode Configured  : " << mode_str << "\n";
    std::cout << " * Frames Processed : TX: " << can_tx << " pkts | RX Verified: " << can_rx << " pkts\n";
    std::cout << " * Errors / Drops   : " << can_err << " errors (" << can_loss_pct << "%)\n";
    std::cout << " * Total CAN Result : ";
    if (can_tx == 0) std::cout << "[NOT TESTED]\n";
    else if (can_err == 0 && can_rx > 0) std::cout << "[ALL PASS - 0 ERRORS]\n";
    else std::cout << "[FAIL - " << can_err << " ERRORS]\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " 4-Port USB Storage Read/Write Stress Detailed Summary:\n";
    for (size_t i = 0; i < engine->usb_tester.disks.size(); ++i) {
        const auto& d = engine->usb_tester.disks[i];
        uint64_t w_mb = d->write_bytes.load() / (1024 * 1024);
        uint64_t r_mb = d->read_bytes.load() / (1024 * 1024);
        uint64_t w_ops = d->write_ops.load();
        uint64_t r_ops = d->read_ops.load();
        uint64_t errs = d->errors.load();

        std::string mod = d->model.empty() ? "USB Storage Device" : d->model;
        char cap_buf[32];
        if (d->capacity_gb > 0) snprintf(cap_buf, sizeof(cap_buf), " (%.1f GB)", d->capacity_gb);
        else cap_buf[0] = '\0';

        std::cout << " * Port " << (i + 1) << " (" << d->dev_name << "): " << mod << cap_buf << "\n";
        std::cout << "   - Mount Point : " << d->mount_point << " (" << d->fs_type << ", Part: " << d->part_name << ")\n";
        if (w_mb == 0 && r_mb == 0 && errs == 0) {
            if (!d->mounted) {
                std::cout << "   - Status      : [NOT CONNECTED / UNPLUGGED]\n";
            } else {
                std::cout << "   - Status      : [NOT TESTED] (Test was not started on this device)\n";
            }
        } else {
            std::cout << "   - Data Tested : Written: " << w_mb << " MB (" << w_ops << " blocks) | Read: " << r_mb << " MB (" << r_ops << " blocks)\n";
            std::cout << "   - Verification: Errors: " << errs;
            if (d->disconnects.load() > 0) {
                std::cout << " (Unplug/IO failures: " << d->disconnects.load() << ", Auto-recovered: " << d->recovers.load() << ")";
            }
            if (errs == 0) {
                std::cout << " [PASS]\n";
            } else {
                std::cout << " [FAIL/ERRORS]\n";
            }
        }
    }
    uint64_t tot_usb_w = engine->usb_tester.get_total_write_mb();
    uint64_t tot_usb_r = engine->usb_tester.get_total_read_mb();
    uint64_t tot_usb_err = engine->usb_tester.get_total_errors();
    std::cout << " Total USB Storage  : Written: " << tot_usb_w << " MB | Read: " << tot_usb_r << " MB | Errors: " << tot_usb_err;
    if (tot_usb_w > 0 && tot_usb_err == 0) std::cout << " [ALL PASS]\n";
    else if (tot_usb_w > 0) std::cout << " [FAIL]\n";
    else std::cout << " [NOT TESTED]\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " Total LAN Ping Sent: " << tot_sent << " packets | Received: " << tot_recv << " | Lost: " << tot_lost << " (" << tot_loss_pct << " %)\n";
    std::cout << " Detailed Log Path  : " << engine->emc_log_path << "\n";
    std::cout << "====================================================================\n\n";
}

void sig_handler(int sig) {
    (void)sig;
    if (g_engine) {
        g_engine->stop_all_stress();
        endwin();
        print_summary_report(g_engine);
        std::cout.flush();
    } else {
        endwin();
    }
    _exit(0);
}

void draw_bar(int y, int x, int width, double percent, int color_pair) {
    double clamped = std::max(0.0, std::min(100.0, percent));
    int bar_chars = std::max(4, width - 8);
    int filled = (int)(bar_chars * clamped / 100.0);
    int empty = bar_chars - filled;

    attron(COLOR_PAIR(7));
    mvaddch(y, x, '[');
    attroff(COLOR_PAIR(7));

    attron(color_pair | A_BOLD);
    for (int i = 0; i < filled; ++i) mvaddch(y, x + 1 + i, ACS_BLOCK);
    attroff(color_pair | A_BOLD);

    attron(COLOR_PAIR(8));
    for (int i = 0; i < empty; ++i) mvaddch(y, x + 1 + filled + i, '.');
    attroff(COLOR_PAIR(8));

    attron(COLOR_PAIR(7));
    mvprintw(y, x + 1 + bar_chars, "]%4.0f%%", percent);
    attroff(COLOR_PAIR(7));
}

void draw_section_banner(int y, int w, int badge_pair, int line_pair, const char* badge, const char* subinfo = "") {
    int x = 2;
    attron(COLOR_PAIR(badge_pair) | A_BOLD);
    mvprintw(y, x, " %s ", badge);
    attroff(COLOR_PAIR(badge_pair) | A_BOLD);
    x += (int)strlen(badge) + 2;

    if (subinfo && strlen(subinfo) > 0) {
        mvprintw(y, x, " ");
        x += 1;
        attron(COLOR_PAIR(7) | A_BOLD);
        mvprintw(y, x, "%s", subinfo);
        attroff(COLOR_PAIR(7) | A_BOLD);
        x += (int)strlen(subinfo);
        mvprintw(y, x, " ");
        x += 1;
    }

    if (x < w - 2) {
        attron(COLOR_PAIR(line_pair));
        mvhline(y, x, '-', w - 2 - x);
        attroff(COLOR_PAIR(line_pair));
    }
}

int get_color_by_load(double pct) {
    if (pct > 80.0) return COLOR_PAIR(3);
    if (pct > 40.0) return COLOR_PAIR(2);
    return COLOR_PAIR(1);
}

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");

    RkTesterEngine engine;
    g_engine = &engine;

    // Command-line arguments
    bool start_in_menu = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << "====================================================================\n"
                      << "       RK3576 Unified Hardware Test Platform (rk_tester)\n"
                      << "====================================================================\n\n"
                      << "[Usage]\n"
                      << "  sudo rk_tester                 (Default target host IP: 192.168.32.100)\n"
                      << "  sudo rk_tester -m              (Launch directly into Test Selection Menu)\n"
                      << "  sudo rk_tester 192.168.32.88   (Specify target host IP: 192.168.32.88)\n"
                      << "  sudo rk_tester -m 192.168.32.88(Menu mode + target host IP)\n\n"
                      << "[Default 4-LAN Configurations]\n"
                      << "  Port 1 (USB / eth0)   : 192.168.32.101\n"
                      << "  Port 2 (PCIe / eth1)  : 192.168.32.102\n"
                      << "  Port 3 (GMAC0 / eth2) : 192.168.32.103\n"
                      << "  Port 4 (GMAC1 / eth3) : 192.168.32.104\n\n"
                      << "[Keyboard Controls]\n"
                      << "  [M] Open Test Items Selection Menu (Interactive Checklist)\n"
                      << "  [1] Toggle CPU 8-Core Compute Stress\n"
                      << "  [2] Toggle Mali-G52 GPU Offscreen Stress\n"
                      << "  [3] Toggle RKNPU Dual-Core 6 TOPS Neural Inference Stress\n"
                      << "  [4] [ALL STRESS] Full Load on CPU + GPU + NPU\n"
                      << "  [5] Toggle 4-LAN Ping & Loss Monitor (100ms)\n"
                      << "  [6] Toggle RS232 Unidirectional Test (ttyS0 -> ttyS1)\n"
                      << "  [7] Toggle RS485 Unidirectional Test (ttyS2 -> ttyS3)\n"
                      << "  [8] Toggle 4-Port USB Storage Read/Write Stress (sda..sdd)\n"
                      << "  [9] Toggle CAN Bus Test (can0 / can1)\n"
                      << "  [D] Switch CAN Direction (CAN0->CAN1 / CAN1->CAN0 / Ping-Pong)\n"
                      << "  [T] Change Target Host IP Dynamically\n"
                      << "  [R] Reset Statistics (All items)\n"
                      << "  [P] Switch CPU/GPU Governor (Performance / Schedutil)\n"
                      << "  [I] Re-initialize 4-LAN Static IPs\n"
                      << "  [Q] Quit & Print Final Test Report\n"
                      << "====================================================================\n";
            return 0;
        } else if (arg == "-m" || arg == "--menu") {
            start_in_menu = true;
        } else if (arg == "--can-0to1") {
            engine.can_tester.set_direction(CanDirection::CAN0_TO_CAN1);
        } else if (arg == "--can-1to0") {
            engine.can_tester.set_direction(CanDirection::CAN1_TO_CAN0);
        } else if (arg == "--can-pingpong") {
            engine.can_tester.set_direction(CanDirection::PING_PONG);
        } else {
            struct sockaddr_in sa;
            if (inet_pton(AF_INET, arg.c_str(), &(sa.sin_addr)) == 1) {
                engine.set_target_ip(arg);
            }
        }
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_GREEN, -1);
        init_pair(2, COLOR_YELLOW, -1);
        init_pair(3, COLOR_RED, -1);
        init_pair(4, COLOR_CYAN, -1);
        init_pair(5, COLOR_MAGENTA, -1);
        init_pair(6, COLOR_BLUE, -1);
        init_pair(7, COLOR_WHITE, -1);
        init_pair(8, COLOR_BLACK, -1);
        // Striking high-contrast section badges
        init_pair(9,  COLOR_BLACK, COLOR_CYAN);    // Black on Cyan: Compute & SoC
        init_pair(10, COLOR_BLACK, COLOR_GREEN);   // Black on Green: Gigabit Ethernet
        init_pair(11, COLOR_BLACK, COLOR_YELLOW);  // Black on Yellow: Fieldbus
        init_pair(12, COLOR_WHITE, COLOR_BLUE);    // White on Blue: USB Storage
        init_pair(13, COLOR_WHITE, COLOR_MAGENTA); // White on Magenta: Highlights
        init_pair(14, COLOR_BLACK, COLOR_WHITE);   // Black on White: Top Header
    }

    engine.get_cpu_loads();
    usleep(200000);

    auto last_ping_send = std::chrono::steady_clock::now();
    auto last_draw = std::chrono::steady_clock::now();
    auto last_net_update = std::chrono::steady_clock::now();
    auto last_usb_update = std::chrono::steady_clock::now();

    bool menu_open = start_in_menu;
    int menu_cursor = 0;
    bool menu_cpu = false;
    bool menu_gpu = false;
    bool menu_npu = false;
    bool menu_emc = false;
    bool menu_rs232 = false;
    bool menu_rs485 = false;
    bool menu_usb = false;
    bool menu_can = false;
    bool menu_ports[4] = {true, true, true, true};

    auto sync_menu_from_engine = [&]() {
        menu_cpu = engine.cpu_stress.load();
        menu_gpu = engine.gpu_stress.load();
        menu_npu = engine.npu_stress.load();
        menu_emc = engine.emc_active.load();
        menu_rs232 = engine.rs232.active.load();
        menu_rs485 = engine.rs485.active.load();
        menu_usb = engine.usb_tester.active.load();
        menu_can = engine.can_tester.active.load();
        for (int i = 0; i < 4; ++i) {
            menu_ports[i] = (i < (int)engine.net_ports.size()) ? engine.net_ports[i].enabled : true;
        }
    };

    std::string prompt_status = start_in_menu ? "Test Items Menu opened. Use [Space] to toggle, [S] to apply."
                                              : "Ready. Press [M] for Menu, [5] LAN-Ping, [6] 232, [7] 485, [8] USB, [9] CAN, [Q] quit.";

    auto apply_menu = [&]() {
        if (menu_cpu != engine.cpu_stress.load()) engine.toggle_cpu_stress();
        if (menu_gpu != engine.gpu_stress.load()) engine.toggle_gpu_stress();
        if (menu_npu != engine.npu_stress.load()) engine.toggle_npu_stress();
        if (menu_emc != engine.emc_active.load()) {
            engine.emc_active.store(menu_emc);
            if (menu_emc) {
                engine.log_emc_event("INFO", "4-LAN Ping test STARTED from Menu towards " + engine.emc_target_ip, 1);
            } else {
                engine.log_emc_event("INFO", "4-LAN Ping test PAUSED from Menu", 2);
            }
        }
        if (menu_rs232 != engine.rs232.active.load()) engine.toggle_rs232();
        if (menu_rs485 != engine.rs485.active.load()) engine.toggle_rs485();
        if (menu_usb != engine.usb_tester.active.load()) engine.toggle_usb_stress();
        if (menu_can != engine.can_tester.active.load()) engine.toggle_can();
        for (size_t i = 0; i < engine.net_ports.size() && i < 4; ++i) {
            engine.net_ports[i].enabled = menu_ports[i];
        }

        int active_cnt = (menu_cpu ? 1 : 0) + (menu_gpu ? 1 : 0) + (menu_npu ? 1 : 0) +
                         (menu_emc ? 1 : 0) + (menu_rs232 ? 1 : 0) + (menu_rs485 ? 1 : 0) +
                         (menu_usb ? 1 : 0) + (menu_can ? 1 : 0);
        prompt_status = "Configuration applied: " + std::to_string(active_cnt) + " test item(s) active.";
        menu_open = false;
    };

    if (menu_open) {
        sync_menu_from_engine();
    }

    while (true) {
        int ch = getch();
        if (ch != ERR) {
            if (menu_open) {
                if (ch == 'q' || ch == 'Q') {
                    break;
                } else if (ch == 27 || ch == 'm' || ch == 'M') {
                    menu_open = false;
                    prompt_status = "Menu closed. No changes applied.";
                } else if (ch == KEY_UP || ch == 'k' || ch == 'K') {
                    menu_cursor = (menu_cursor + 14 - 1) % 14;
                } else if (ch == KEY_DOWN || ch == 'j' || ch == 'J') {
                    menu_cursor = (menu_cursor + 1) % 14;
                } else if (ch == KEY_LEFT) {
                    if (menu_cursor >= 9 && menu_cursor <= 11) menu_cursor--;
                    else if (menu_cursor == 13) menu_cursor = 12;
                } else if (ch == KEY_RIGHT) {
                    if (menu_cursor >= 8 && menu_cursor <= 10) menu_cursor++;
                    else if (menu_cursor == 12) menu_cursor = 13;
                } else if (ch == ' ' || ch == '\n' || ch == '\r' || ch == KEY_ENTER) {
                    if (menu_cursor == 0) menu_cpu = !menu_cpu;
                    else if (menu_cursor == 1) menu_gpu = !menu_gpu;
                    else if (menu_cursor == 2) menu_npu = !menu_npu;
                    else if (menu_cursor == 3) menu_emc = !menu_emc;
                    else if (menu_cursor == 4) menu_rs232 = !menu_rs232;
                    else if (menu_cursor == 5) menu_rs485 = !menu_rs485;
                    else if (menu_cursor == 6) menu_usb = !menu_usb;
                    else if (menu_cursor == 7) menu_can = !menu_can;
                    else if (menu_cursor == 8) menu_ports[0] = !menu_ports[0];
                    else if (menu_cursor == 9) menu_ports[1] = !menu_ports[1];
                    else if (menu_cursor == 10) menu_ports[2] = !menu_ports[2];
                    else if (menu_cursor == 11) menu_ports[3] = !menu_ports[3];
                    else if (menu_cursor == 12) apply_menu();
                    else if (menu_cursor == 13) {
                        menu_open = false;
                        prompt_status = "Menu closed. No changes applied.";
                    }
                } else if (ch == '1') {
                    menu_cpu = !menu_cpu; menu_cursor = 0;
                } else if (ch == '2') {
                    menu_gpu = !menu_gpu; menu_cursor = 1;
                } else if (ch == '3') {
                    menu_npu = !menu_npu; menu_cursor = 2;
                } else if (ch == '4') {
                    menu_cpu = true; menu_gpu = true; menu_npu = true;
                } else if (ch == '5') {
                    menu_emc = !menu_emc; menu_cursor = 3;
                } else if (ch == '6') {
                    menu_rs232 = !menu_rs232; menu_cursor = 4;
                } else if (ch == '7') {
                    menu_rs485 = !menu_rs485; menu_cursor = 5;
                } else if (ch == '8') {
                    menu_usb = !menu_usb; menu_cursor = 6;
                } else if (ch == '9') {
                    menu_can = !menu_can; menu_cursor = 7;
                } else if (ch == 'd' || ch == 'D') {
                    engine.can_tester.cycle_direction();
                    prompt_status = "CAN mode switched: " + engine.can_tester.get_mode_name();
                } else if (ch == 'a' || ch == 'A') {
                    menu_cpu = true; menu_gpu = true; menu_npu = true;
                    menu_emc = true; menu_rs232 = true; menu_rs485 = true;
                    menu_usb = true; menu_can = true;
                    menu_ports[0] = true; menu_ports[1] = true;
                    menu_ports[2] = true; menu_ports[3] = true;
                } else if (ch == 'n' || ch == 'N') {
                    menu_cpu = false; menu_gpu = false; menu_npu = false;
                    menu_emc = false; menu_rs232 = false; menu_rs485 = false;
                    menu_usb = false; menu_can = false;
                } else if (ch == 's' || ch == 'S') {
                    apply_menu();
                }
            } else {
                if (ch == 'q' || ch == 'Q' || ch == 27) {
                    break;
                } else if (ch == 'm' || ch == 'M') {
                    menu_open = true;
                    menu_cursor = 0;
                    sync_menu_from_engine();
                    prompt_status = "Test Items Menu opened. Use [Up/Down], [Space], [S] to apply.";
                } else if (ch == '1' || ch == 'c' || ch == 'C') {
                    engine.toggle_cpu_stress();
                    prompt_status = engine.cpu_stress.load() ? "CPU 8-core compute stress STARTED." : "CPU stress STOPPED.";
                } else if (ch == '2' || ch == 'g' || ch == 'G') {
                    engine.toggle_gpu_stress();
                    prompt_status = engine.gpu_stress.load() ? "Mali-G52 GPU offscreen stress STARTED." : "GPU stress STOPPED.";
                } else if (ch == '3' || ch == 'n' || ch == 'N') {
                    engine.toggle_npu_stress();
                    prompt_status = engine.npu_stress.load() ? "RKNPU dual-core 6 TOPS stress STARTED." : "NPU stress STOPPED.";
                } else if (ch == '4' || ch == 'a' || ch == 'A') {
                    engine.toggle_all_stress();
                    bool on = engine.cpu_stress.load() && engine.gpu_stress.load() && engine.npu_stress.load();
                    prompt_status = on ? "ALL STRESS ON: CPU + GPU + NPU running at full load!" : "All stress STOPPED.";
                } else if (ch == '5' || ch == 'e' || ch == 'E') {
                    engine.emc_active.store(!engine.emc_active.load());
                    if (engine.emc_active.load()) {
                        prompt_status = "4-LAN ping loss monitoring STARTED (100ms).";
                        engine.log_emc_event("INFO", "4-LAN Ping test manually STARTED towards " + engine.emc_target_ip, 1);
                    } else {
                        prompt_status = "4-LAN ping monitoring PAUSED.";
                        engine.log_emc_event("INFO", "4-LAN Ping test manually PAUSED.", 2);
                    }
                } else if (ch == '6') {
                    engine.toggle_rs232();
                    prompt_status = engine.rs232.active.load() ? "RS232 test (ttyS0->ttyS1) STARTED." : "RS232 test STOPPED.";
                } else if (ch == '7') {
                    engine.toggle_rs485();
                    prompt_status = engine.rs485.active.load() ? "RS485 test (ttyS2->ttyS3) STARTED." : "RS485 test STOPPED.";
                } else if (ch == '8' || ch == 'u' || ch == 'U') {
                    engine.toggle_usb_stress();
                    prompt_status = engine.usb_tester.active.load() ? "4-Port USB storage R/W stress STARTED." : "USB storage stress STOPPED.";
                } else if (ch == '9') {
                    engine.toggle_can();
                    prompt_status = engine.can_tester.active.load() ? ("CAN (" + engine.can_tester.get_mode_name() + ") STARTED.") : "CAN test STOPPED.";
                } else if (ch == 'd' || ch == 'D') {
                    engine.can_tester.cycle_direction();
                    prompt_status = "CAN mode switched to: " + engine.can_tester.get_mode_name();
                } else if (ch == 't' || ch == 'T') {
                    // Interactive target IP modification
                    echo();
                    curs_set(1);
                    int h = 0, w = 0;
                    getmaxyx(stdscr, h, w);
                    (void)w;
                    mvprintw(h - 1, 2, "Enter new target host IP (e.g. 192.168.32.88, press ENTER): ");
                    clrtoeol();
                    refresh();
                    char input_ip[64];
                    getnstr(input_ip, sizeof(input_ip) - 1);
                    noecho();
                    curs_set(0);

                    std::string new_ip = input_ip;
                    while (!new_ip.empty() && (new_ip.back() == ' ' || new_ip.back() == '\r' || new_ip.back() == '\n')) new_ip.pop_back();
                    while (!new_ip.empty() && new_ip.front() == ' ') new_ip.erase(0, 1);

                    struct sockaddr_in sa;
                    if (inet_pton(AF_INET, new_ip.c_str(), &(sa.sin_addr)) == 1) {
                        engine.set_target_ip(new_ip);
                        engine.log_emc_event("INFO", "Target host IP updated to: " + new_ip, 4);
                        prompt_status = "Target IP successfully updated to: " + new_ip;
                    } else {
                        prompt_status = "Invalid IP format. Keeping current IP: " + engine.emc_target_ip;
                    }
                } else if (ch == 'i' || ch == 'I') {
                    engine.setup_default_network();
                    prompt_status = "4-LAN static IPs re-initialized (192.168.32.101~104).";
                } else if (ch == 'p' || ch == 'P') {
                    engine.toggle_governor();
                    prompt_status = engine.governor_perf ? "Governor locked to PERFORMANCE mode." : "Governor restored to SCHEDUTIL (dynamic) mode.";
                } else if (ch == 'r' || ch == 'R') {
                    engine.reset_emc_stats();
                    prompt_status = "All statistics (EMC & Serial) reset to zero.";
                }
            }
        }

        auto now = std::chrono::steady_clock::now();

        // 1. EMC 发包
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ping_send).count() >= engine.emc_interval_ms) {
            last_ping_send = now;
            engine.emc_tick_send();
        }

        // 2. EMC 收包与超时检测
        engine.emc_tick_recv();
        engine.emc_tick_timeout();

        // 3. 定时更新网卡物理链路状态 (每300ms)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_net_update).count() >= 300) {
            last_net_update = now;
            engine.update_net_status();
        }

        // 4. 定时检测 USB 存储热插拔 (每1000ms)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_usb_update).count() >= 1000) {
            last_usb_update = now;
            engine.usb_tester.check_and_update_disks();
        }

        // 4. 定时渲染 TUI 界面 (每100ms)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_draw).count() >= 100) {
            last_draw = now;

            auto [total_cpu, core_cpus] = engine.get_cpu_loads();
            auto cpu_freqs = engine.get_cpu_freqs();
            auto [gpu_load, gpu_freq] = engine.get_gpu_stats();
            auto [npu_loads, npu_freq] = engine.get_npu_stats();
            auto [used_ram, tot_ram, ram_pct] = engine.get_mem_stats();
            auto thermals = engine.get_thermals();
            std::string uptime = engine.get_uptime();

            erase();
            int h = 0, w = 0;
            getmaxyx(stdscr, h, w);

            if (h < 23 || w < 76) {
                attron(COLOR_PAIR(3) | A_BOLD);
                mvprintw(0, 0, "Terminal window too small! Please resize (Need >= 76x23, current %dx%d)", w, h);
                attroff(COLOR_PAIR(3) | A_BOLD);
                refresh();
                usleep(30000);
                continue;
            }

            if (menu_open) {
                int box_w = 72;
                int box_h = 18;
                int my = std::max(1, (h - box_h) / 2);
                int mx = std::max(2, (w - box_w) / 2);

                // Top title bar
                attron(COLOR_PAIR(4) | A_BOLD);
                mvprintw(my, mx, "+----------------------------------------------------------------------+");
                mvprintw(my + 1, mx, "|               TEST ITEMS CONFIGURATION & CHECKLIST                   |");
                mvprintw(my + 2, mx, "+----------------------------------------------------------------------+");
                attroff(COLOR_PAIR(4) | A_BOLD);

                attron(COLOR_PAIR(7));
                mvprintw(my + 3, mx, "| Use [Up/Down], [Space] to toggle, [1..9] quick toggle, [D] CAN Dir:  |");
                mvprintw(my + 4, mx, "|                                                                      |");
                attroff(COLOR_PAIR(7));

                struct MenuItem {
                    int id;
                    std::string key_str;
                    std::string name;
                    std::string desc;
                    bool checked;
                };

                MenuItem items[8] = {
                    {0, "[1]", "CPU 8-Core Compute Stress", "8-Core full compute burn-in", menu_cpu},
                    {1, "[2]", "Mali-G52 GPU 3D Stress   ", "Offscreen 3D rendering stress", menu_gpu},
                    {2, "[3]", "RK-NPU Dual-Core AI Stress", "Dual-Core 6 TOPS neural burn", menu_npu},
                    {3, "[5]", "4-LAN Ping & Loss Monitor", "100ms ICMP packet loss check", menu_emc},
                    {4, "[6]", "RS232 Serial Loopback    ", "ttyS0 -> ttyS1 (115200 8N1)", menu_rs232},
                    {5, "[7]", "RS485 Serial Loopback    ", "ttyS2 -> ttyS3 (115200 8N1)", menu_rs485},
                    {6, "[8]", "4-Port USB Storage Stress", "sda..sdd read/write & verify", menu_usb},
                    {7, "[9]", "CAN Bus Stress (" + engine.can_tester.get_short_mode_name() + ")", engine.can_tester.get_mode_name() + " [D]", menu_can}
                };

                for (int i = 0; i < 8; ++i) {
                    int cur_y = my + 5 + i;
                    attron(COLOR_PAIR(4) | A_BOLD);
                    mvprintw(cur_y, mx, "|");
                    mvprintw(cur_y, mx + box_w - 1, "|");
                    attroff(COLOR_PAIR(4) | A_BOLD);

                    bool is_sel = (menu_cursor == i);
                    if (is_sel) {
                        attron(COLOR_PAIR(2) | A_BOLD);
                        mvprintw(cur_y, mx + 2, "->");
                        attroff(COLOR_PAIR(2) | A_BOLD);
                    } else {
                        mvprintw(cur_y, mx + 2, "  ");
                    }

                    // Checkbox [X] or [ ]
                    if (items[i].checked) {
                        attron(COLOR_PAIR(1) | A_BOLD);
                        mvprintw(cur_y, mx + 5, "[X]");
                        attroff(COLOR_PAIR(1) | A_BOLD);
                    } else {
                        attron(COLOR_PAIR(8) | A_BOLD);
                        mvprintw(cur_y, mx + 5, "[ ]");
                        attroff(COLOR_PAIR(8) | A_BOLD);
                    }

                    attron(COLOR_PAIR(6) | A_BOLD);
                    mvprintw(cur_y, mx + 9, "%s", items[i].key_str.c_str());
                    attroff(COLOR_PAIR(6) | A_BOLD);

                    int text_col = is_sel ? (COLOR_PAIR(7) | A_BOLD | A_UNDERLINE) : COLOR_PAIR(7);
                    attron(text_col);
                    mvprintw(cur_y, mx + 13, " %-27s", items[i].name.c_str());
                    attroff(text_col);

                    attron(COLOR_PAIR(8));
                    mvprintw(cur_y, mx + 42, "(%s)", items[i].desc.c_str());
                    attroff(COLOR_PAIR(8));
                }

                // Line my + 13: LAN Ports Filter
                int py = my + 13;
                attron(COLOR_PAIR(4) | A_BOLD);
                mvprintw(py, mx, "|");
                mvprintw(py, mx + box_w - 1, "|");
                attroff(COLOR_PAIR(4) | A_BOLD);

                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(py, mx + 3, "LAN Ports Filter (LAN 0..3):");
                attroff(COLOR_PAIR(7) | A_BOLD);

                const char* p_names[4] = {"eth0", "eth1", "eth2", "eth3"};
                int px = mx + 36;
                for (int i = 0; i < 4; ++i) {
                    bool is_sel = (menu_cursor == 8 + i);
                    if (is_sel) {
                        attron(COLOR_PAIR(2) | A_BOLD);
                        mvprintw(py, px - 1, ">");
                        attroff(COLOR_PAIR(2) | A_BOLD);
                    }
                    if (menu_ports[i]) {
                        attron(COLOR_PAIR(1) | A_BOLD);
                        mvprintw(py, px, "[X]%s", p_names[i]);
                        attroff(COLOR_PAIR(1) | A_BOLD);
                    } else {
                        attron(COLOR_PAIR(8) | A_BOLD);
                        mvprintw(py, px, "[ ]%s", p_names[i]);
                        attroff(COLOR_PAIR(8) | A_BOLD);
                    }
                    px += 8;
                }

                // Line my + 14: Action Buttons
                int ay = my + 14;
                attron(COLOR_PAIR(4) | A_BOLD);
                mvprintw(ay, mx, "|");
                mvprintw(ay, mx + box_w - 1, "|");
                attroff(COLOR_PAIR(4) | A_BOLD);

                bool sel_apply = (menu_cursor == 12);
                bool sel_cancel = (menu_cursor == 13);

                if (sel_apply) {
                    attron(COLOR_PAIR(1) | A_REVERSE | A_BOLD);
                    mvprintw(ay, mx + 5, "[ S / ENTER: Apply & Run Selected Tests ]");
                    attroff(COLOR_PAIR(1) | A_REVERSE | A_BOLD);
                } else {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    mvprintw(ay, mx + 5, "[ S / ENTER: Apply & Run Selected Tests ]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                }

                if (sel_cancel) {
                    attron(COLOR_PAIR(3) | A_REVERSE | A_BOLD);
                    mvprintw(ay, mx + 49, "[ ESC: Cancel ]");
                    attroff(COLOR_PAIR(3) | A_REVERSE | A_BOLD);
                } else {
                    attron(COLOR_PAIR(7));
                    mvprintw(ay, mx + 49, "[ ESC: Cancel ]");
                    attroff(COLOR_PAIR(7));
                }

                // Line my + 15: Help row
                int hy = my + 15;
                attron(COLOR_PAIR(4) | A_BOLD);
                mvprintw(hy, mx, "|");
                mvprintw(hy, mx + box_w - 1, "|");
                attroff(COLOR_PAIR(4) | A_BOLD);
                attron(COLOR_PAIR(7));
                mvprintw(hy, mx + 3, "Hotkeys: ");
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(2) | A_BOLD);
                mvprintw(hy, mx + 12, "[A]");
                attroff(COLOR_PAIR(2) | A_BOLD);
                attron(COLOR_PAIR(7));
                mvprintw(hy, mx + 15, "Check All  ");
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(2) | A_BOLD);
                mvprintw(hy, mx + 26, "[N]");
                attroff(COLOR_PAIR(2) | A_BOLD);
                attron(COLOR_PAIR(7));
                mvprintw(hy, mx + 29, "Clear All  ");
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(2) | A_BOLD);
                mvprintw(hy, mx + 40, "[S]");
                attroff(COLOR_PAIR(2) | A_BOLD);
                attron(COLOR_PAIR(7));
                mvprintw(hy, mx + 43, "Apply  ");
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(2) | A_BOLD);
                mvprintw(hy, mx + 50, "[ESC/M]");
                attroff(COLOR_PAIR(2) | A_BOLD);
                attron(COLOR_PAIR(7));
                mvprintw(hy, mx + 57, "Back to Dashboard");
                attroff(COLOR_PAIR(7));

                // Bottom border
                attron(COLOR_PAIR(4) | A_BOLD);
                mvprintw(my + 16, mx, "+----------------------------------------------------------------------+");
                attroff(COLOR_PAIR(4) | A_BOLD);

                // Bottom Status Prompt
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(my + 17, mx, "Status: %-62s", prompt_status.c_str());
                attroff(COLOR_PAIR(7) | A_BOLD);

                refresh();
                continue;
            }

            // Top title
            attron(COLOR_PAIR(14) | A_BOLD);
            mvprintw(0, 2, " RK3576 INDUSTRIAL HARDWARE TEST PLATFORM ");
            attroff(COLOR_PAIR(14) | A_BOLD);

            char meta[128];
            snprintf(meta, sizeof(meta), "Uptime: %s | Gov: %s",
                     uptime.c_str(), engine.governor_perf ? "PERF" : "DYNAMIC");
            attron(COLOR_PAIR(7));
            mvprintw(0, std::max(48, w - (int)strlen(meta) - 3), "%s", meta);
            attroff(COLOR_PAIR(7));

            bool spacious = (h >= 30);
            int line = 1;
            if (spacious) {
                attron(COLOR_PAIR(4));
                mvhline(1, 2, '=', w - 4);
                attroff(COLOR_PAIR(4));
                line = 2;
            }

            // ====================================================================
            // SECTION 1: COMPUTE & SOC ACCELERATORS (CPU, GPU, NPU, RAM, Temp)
            // ====================================================================
            draw_section_banner(line++, w, 9, 4, "[1..3] COMPUTE & SOC ACCELERATORS", "CPU / GPU / NPU / Memory / Temp");

            if (spacious) {
                // Row 1: CPU Total Load bar
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "CPU Stress:");
                attroff(COLOR_PAIR(7) | A_BOLD);

                std::string cpu_st = engine.cpu_stress.load() ? "[STRESS ON: 8-CORES]" : "[IDLE]";
                int cpu_col = engine.cpu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
                attron(cpu_col);
                mvprintw(line, 16, "%s", cpu_st.c_str());
                attroff(cpu_col);

                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 39, "Total Load: ");
                attroff(COLOR_PAIR(7) | A_BOLD);
                draw_bar(line, 51, w - 54, total_cpu, get_color_by_load(total_cpu));
                line++;

                // Rows 2..5: 4 rows for A53 & A72 cores with graphical bars
                int half_w = (w - 4) / 2;
                int bar_w = half_w - 20;
                for (int i = 0; i < 4; ++i) {
                    // Little cores A53
                    int f_a53 = (i < (int)cpu_freqs.size()) ? cpu_freqs[i] : 0;
                    double p_a53 = (i < (int)core_cpus.size()) ? core_cpus[i] : 0.0;
                    attron(COLOR_PAIR(7));
                    mvprintw(line + i, 3, "A53 Core C%d:", i);
                    attroff(COLOR_PAIR(7));
                    draw_bar(line + i, 15, bar_w, p_a53, get_color_by_load(p_a53));
                    attron(COLOR_PAIR(6));
                    mvprintw(line + i, 15 + bar_w + 1, "%4dM", f_a53);
                    attroff(COLOR_PAIR(6));

                    // Big cores A72
                    int bi = i + 4;
                    int f_a72 = (bi < (int)cpu_freqs.size()) ? cpu_freqs[bi] : 0;
                    double p_a72 = (bi < (int)core_cpus.size()) ? core_cpus[bi] : 0.0;
                    attron(COLOR_PAIR(7));
                    mvprintw(line + i, half_w + 2, "A72 Core C%d:", bi);
                    attroff(COLOR_PAIR(7));
                    draw_bar(line + i, half_w + 14, bar_w, p_a72, get_color_by_load(p_a72));
                    attron(COLOR_PAIR(6));
                    mvprintw(line + i, half_w + 14 + bar_w + 1, "%4dM", f_a72);
                    attroff(COLOR_PAIR(6));
                }
                line += 4;

                // Row 6: GPU & NPU Accelerators
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "Mali GPU:");
                attroff(COLOR_PAIR(7) | A_BOLD);
                std::string gpu_st = engine.gpu_stress.load() ? "[RUN]" : "[IDLE]";
                int gpu_col = engine.gpu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
                attron(gpu_col);
                mvprintw(line, 13, "%s", gpu_st.c_str());
                attroff(gpu_col);
                draw_bar(line, 20, 14, gpu_load, get_color_by_load(gpu_load));
                attron(COLOR_PAIR(6));
                mvprintw(line, 35, "%4dM", gpu_freq);
                attroff(COLOR_PAIR(6));

                attron(COLOR_PAIR(7));
                mvprintw(line, 41, "| NPU:");
                attroff(COLOR_PAIR(7));
                std::string npu_st = engine.npu_stress.load() ? "[RUN]" : "[IDLE]";
                int npu_col = engine.npu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
                attron(npu_col);
                mvprintw(line, 48, "%s", npu_st.c_str());
                attroff(npu_col);
                attron(COLOR_PAIR(7));
                mvprintw(line, 55, "C0:%3.0f%% C1:%3.0f%%", npu_loads[0], npu_loads[1]);
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(6));
                mvprintw(line, 71, "%4dM", npu_freq);
                attroff(COLOR_PAIR(6));
                line++;

                // Row 7: RAM & Temperature
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "RAM Load:");
                attroff(COLOR_PAIR(7) | A_BOLD);
                attron(COLOR_PAIR(7));
                mvprintw(line, 13, "%3.1fG/%3.1fG", used_ram, tot_ram);
                attroff(COLOR_PAIR(7));
                draw_bar(line, 24, 12, ram_pct, get_color_by_load(ram_pct));

                attron(COLOR_PAIR(5) | A_BOLD);
                mvprintw(line, 40, "| Temp: SoC:%.0fC Big:%.0fC GPU:%.0fC NPU:%.0fC",
                         thermals["soc"], thermals["bigcore"], thermals["gpu"], thermals["npu"]);
                attroff(COLOR_PAIR(5) | A_BOLD);
                line++;
            } else {
                // Compact mode (h < 30): 4 lines
                // Row 1: CPU Total + RAM + Temp
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "CPU:");
                attroff(COLOR_PAIR(7) | A_BOLD);

                std::string cpu_st = engine.cpu_stress.load() ? "[STRESS ON]" : "[IDLE]";
                int cpu_col = engine.cpu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
                attron(cpu_col);
                mvprintw(line, 8, "%s", cpu_st.c_str());
                attroff(cpu_col);

                draw_bar(line, 20, 12, total_cpu, get_color_by_load(total_cpu));

                attron(COLOR_PAIR(7));
                mvprintw(line, 33, "| RAM:%3.1fG/%3.1fG", used_ram, tot_ram);
                attroff(COLOR_PAIR(7));

                attron(COLOR_PAIR(5) | A_BOLD);
                mvprintw(line, 52, "| Temp:SoC:%.0fC Big:%.0fC", thermals["soc"], thermals["bigcore"]);
                attroff(COLOR_PAIR(5) | A_BOLD);
                line++;

                // Row 2: A53 Cores
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "A53:");
                attroff(COLOR_PAIR(7) | A_BOLD);
                int cx = 8;
                for (int i = 0; i < 4; ++i) {
                    double p = (i < (int)core_cpus.size()) ? core_cpus[i] : 0.0;
                    int f = (i < (int)cpu_freqs.size()) ? cpu_freqs[i] : 0;
                    attron(COLOR_PAIR(7));
                    mvprintw(line, cx, "C%d:", i);
                    attroff(COLOR_PAIR(7));
                    attron(get_color_by_load(p) | A_BOLD);
                    mvprintw(line, cx + 3, "%2.0f%%", p);
                    attroff(get_color_by_load(p) | A_BOLD);
                    attron(COLOR_PAIR(6));
                    mvprintw(line, cx + 6, "(%dM)", f);
                    attroff(COLOR_PAIR(6));
                    cx += 17;
                }
                line++;

                // Row 3: A72 Cores
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "A72:");
                attroff(COLOR_PAIR(7) | A_BOLD);
                cx = 8;
                for (int i = 0; i < 4; ++i) {
                    int bi = i + 4;
                    double p = (bi < (int)core_cpus.size()) ? core_cpus[bi] : 0.0;
                    int f = (bi < (int)cpu_freqs.size()) ? cpu_freqs[bi] : 0;
                    attron(COLOR_PAIR(7));
                    mvprintw(line, cx, "C%d:", bi);
                    attroff(COLOR_PAIR(7));
                    attron(get_color_by_load(p) | A_BOLD);
                    mvprintw(line, cx + 3, "%2.0f%%", p);
                    attroff(get_color_by_load(p) | A_BOLD);
                    attron(COLOR_PAIR(6));
                    mvprintw(line, cx + 6, "(%dM)", f);
                    attroff(COLOR_PAIR(6));
                    cx += 17;
                }
                line++;

                // Row 4: GPU & NPU
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "GPU:");
                attroff(COLOR_PAIR(7) | A_BOLD);
                std::string gpu_st = engine.gpu_stress.load() ? "[RUN]" : "[IDLE]";
                int gpu_col = engine.gpu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
                attron(gpu_col);
                mvprintw(line, 8, "%s", gpu_st.c_str());
                attroff(gpu_col);
                draw_bar(line, 15, 12, gpu_load, get_color_by_load(gpu_load));
                attron(COLOR_PAIR(6));
                mvprintw(line, 28, "%4dM", gpu_freq);
                attroff(COLOR_PAIR(6));

                attron(COLOR_PAIR(7));
                mvprintw(line, 34, "| NPU:");
                attroff(COLOR_PAIR(7));
                std::string npu_st = engine.npu_stress.load() ? "[RUN]" : "[IDLE]";
                int npu_col = engine.npu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
                attron(npu_col);
                mvprintw(line, 41, "%s", npu_st.c_str());
                attroff(npu_col);
                attron(COLOR_PAIR(7));
                mvprintw(line, 48, "C0:%2.0f%% C1:%2.0f%%", npu_loads[0], npu_loads[1]);
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(6));
                mvprintw(line, 62, "%4dM", npu_freq);
                attroff(COLOR_PAIR(6));
                attron(COLOR_PAIR(5) | A_BOLD);
                mvprintw(line, 68, "G:%.0fC", thermals["gpu"]);
                attroff(COLOR_PAIR(5) | A_BOLD);
                line++;
            }

            // Spacing before Section 2
            line++;

            // ====================================================================
            // SECTION 2: 4-PORT GIGABIT ETHERNET & PING MONITOR
            // ====================================================================
            draw_section_banner(line++, w, 10, 1, "[5] 4-PORT GIGABIT ETHERNET", "Link Status & 100ms Ping EMC Monitor");

            for (int i = 0; i < 4; ++i) {
                const auto& port = engine.net_ports[i];
                attron(COLOR_PAIR(2));
                mvprintw(line + i, 3, "%-18s", port.ch_name.c_str());
                attroff(COLOR_PAIR(2));

                if (!port.enabled) {
                    attron(COLOR_PAIR(8) | A_BOLD);
                    mvprintw(line + i, 22, "[ DISABLED  ]");
                    attroff(COLOR_PAIR(8) | A_BOLD);
                } else if (port.carrier) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    mvprintw(line + i, 22, "[ UP %4dM ]", port.speed > 0 ? port.speed : 0);
                    attroff(COLOR_PAIR(1) | A_BOLD);
                } else {
                    attron(COLOR_PAIR(3));
                    mvprintw(line + i, 22, "[ LINK DOWN ]");
                    attroff(COLOR_PAIR(3));
                }

                std::string ip_suffix = port.current_ip;
                size_t dot_pos = ip_suffix.rfind('.');
                if (dot_pos != std::string::npos) ip_suffix = ip_suffix.substr(dot_pos);
                else ip_suffix = "---";

                attron(COLOR_PAIR(7));
                mvprintw(line + i, 35, "%-4s | S:%-4llu R:%-4llu Lost:",
                         ip_suffix.c_str(),
                         (unsigned long long)port.sent, (unsigned long long)port.recv);
                attroff(COLOR_PAIR(7));

                int p_lost_col = (port.lost == 0) ? COLOR_PAIR(1) : (COLOR_PAIR(3) | A_BOLD);
                double p_loss_pct = (port.sent > 0) ? (port.lost * 100.0 / port.sent) : 0.0;
                attron(p_lost_col);
                mvprintw(line + i, 58, "%-3llu", (unsigned long long)port.lost);
                attroff(p_lost_col);

                attron(COLOR_PAIR(7));
                if (port.sent > 0) {
                    mvprintw(line + i, 62, "(%4.1f%%)", p_loss_pct);
                } else {
                    mvprintw(line + i, 62, "( --- )");
                }
                if (port.recv > 0) {
                    mvprintw(line + i, 71, "%4.1fms", port.rtt_cur);
                } else {
                    mvprintw(line + i, 71, " --.-ms");
                }
                attroff(COLOR_PAIR(7));
            }
            line += 4;

            // Ping stats summary
            uint64_t tot_sent = engine.get_total_sent();
            uint64_t tot_recv = engine.get_total_recv();
            uint64_t tot_lost = engine.get_total_lost();
            uint64_t tot_flaps = engine.get_total_flaps();
            double tot_loss_pct = (tot_sent > 0) ? (tot_lost * 100.0 / tot_sent) : 0.0;

            std::string rating = "STANDBY";
            int rate_col = COLOR_PAIR(7);
            if (tot_sent > 0) {
                if (tot_flaps > 0) {
                    rating = "CLASS C (FAIL)";
                    rate_col = COLOR_PAIR(3) | A_BOLD;
                } else if (tot_lost > 0) {
                    if (tot_loss_pct < 1.0) {
                        rating = "CLASS B (<1%)";
                        rate_col = COLOR_PAIR(2) | A_BOLD;
                    } else {
                        rating = "CLASS C (FAIL)";
                        rate_col = COLOR_PAIR(3) | A_BOLD;
                    }
                } else {
                    rating = "CLASS A (PASS)";
                    rate_col = COLOR_PAIR(1) | A_BOLD;
                }
            }

            std::string emc_st = engine.emc_active.load() ? "[RUN 100ms]" : "[STANDBY]";
            int emc_col = engine.emc_active.load() ? (COLOR_PAIR(1) | A_BOLD) : COLOR_PAIR(2);

            if (spacious) {
                // Line 1: Header + Status + Rating + Target
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "EMC Ping:");
                attroff(COLOR_PAIR(7) | A_BOLD);
                attron(emc_col);
                mvprintw(line, 13, "%s", emc_st.c_str());
                attroff(emc_col);
                attron(rate_col);
                mvprintw(line, 26, "[%s]", rating.c_str());
                attroff(rate_col);
                attron(COLOR_PAIR(7));
                mvprintw(line, 45, "Target IP: %-15s", engine.emc_target_ip.c_str());
                attroff(COLOR_PAIR(7));
                line++;

                // Line 2: Detailed Ping Stats
                attron(COLOR_PAIR(7));
                mvprintw(line, 3, "Ping Stats: Sent:%-5llu Recv:%-5llu Lost:",
                         (unsigned long long)tot_sent, (unsigned long long)tot_recv);
                attroff(COLOR_PAIR(7));
                int lost_col = (tot_lost == 0) ? COLOR_PAIR(1) : (COLOR_PAIR(3) | A_BOLD);
                attron(lost_col);
                mvprintw(line, 44, "%-4llu", (unsigned long long)tot_lost);
                attroff(lost_col);
                attron(COLOR_PAIR(7));
                mvprintw(line, 49, "(%4.1f%%) | Link Flaps:%-2llu", tot_loss_pct, (unsigned long long)tot_flaps);
                attroff(COLOR_PAIR(7));
                line++;
            } else {
                // Compact mode (single row, strictly fitted within 78 cols)
                attron(COLOR_PAIR(7) | A_BOLD);
                mvprintw(line, 3, "Ping:");
                attroff(COLOR_PAIR(7) | A_BOLD);
                attron(emc_col);
                mvprintw(line, 9, "%s", emc_st.c_str());
                attroff(emc_col);
                attron(rate_col);
                mvprintw(line, 21, "[%s]", rating.c_str());
                attroff(rate_col);
                attron(COLOR_PAIR(7));
                std::string short_tgt = engine.emc_target_ip;
                mvprintw(line, 37, "Tgt:%-14s | S:%-4llu L:", short_tgt.c_str(), (unsigned long long)tot_sent);
                attroff(COLOR_PAIR(7));
                int lost_col = (tot_lost == 0) ? COLOR_PAIR(1) : (COLOR_PAIR(3) | A_BOLD);
                attron(lost_col);
                mvprintw(line, 66, "%-3llu", (unsigned long long)tot_lost);
                attroff(lost_col);
                attron(COLOR_PAIR(7));
                mvprintw(line, 70, "Flap:%-2llu", (unsigned long long)tot_flaps);
                attroff(COLOR_PAIR(7));
                line++;
            }

            if (spacious) {
                attron(COLOR_PAIR(7));
                mvprintw(line, 3, "Latest Event: ");
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(engine.emc_latest_event_color) | A_BOLD);
                std::string disp_ev = engine.emc_latest_event;
                int max_ev_len = w - 18;
                if ((int)disp_ev.length() > max_ev_len) disp_ev = disp_ev.substr(0, max_ev_len);
                mvprintw(line, 17, "%s", disp_ev.c_str());
                attroff(COLOR_PAIR(engine.emc_latest_event_color) | A_BOLD);
                line++;
            }

            // Spacing before Section 3
            line++;

            // ====================================================================
            // SECTION 3: INDUSTRIAL FIELDBUS (SERIAL & CAN)
            // ====================================================================
            draw_section_banner(line++, w, 11, 2, "[6,7,9] INDUSTRIAL FIELDBUS", "RS232 / RS485 / CAN Bus Testing");

            // Row 1: RS232 + RS485
            bool a232 = engine.rs232.active.load();
            uint64_t e232 = engine.rs232.err_frames.load();
            double spd232 = engine.rs232.speed_kb.load();
            std::string st232 = a232 ? "[RUN]" : "[IDLE]";
            int c232 = a232 ? ((e232 == 0) ? (COLOR_PAIR(1) | A_BOLD) : (COLOR_PAIR(3) | A_BOLD)) : COLOR_PAIR(7);

            attron(COLOR_PAIR(7) | A_BOLD);
            mvprintw(line, 3, "RS232(S0->S1):");
            attroff(COLOR_PAIR(7) | A_BOLD);
            attron(c232);
            mvprintw(line, 18, "%s", st232.c_str());
            attroff(c232);
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(line, 25, "%4.1fK/s", spd232);
            attroff(COLOR_PAIR(6) | A_BOLD);
            attron(c232);
            mvprintw(line, 34, "(%lluE)", (unsigned long long)e232);
            attroff(c232);

            attron(COLOR_PAIR(7));
            mvprintw(line, 40, "|");
            attroff(COLOR_PAIR(7));

            bool a485 = engine.rs485.active.load();
            uint64_t e485 = engine.rs485.err_frames.load();
            double spd485 = engine.rs485.speed_kb.load();
            std::string st485 = a485 ? "[RUN]" : "[IDLE]";
            int c485 = a485 ? ((e485 == 0) ? (COLOR_PAIR(1) | A_BOLD) : (COLOR_PAIR(3) | A_BOLD)) : COLOR_PAIR(7);

            attron(COLOR_PAIR(7) | A_BOLD);
            mvprintw(line, 42, "RS485(S2->S3):");
            attroff(COLOR_PAIR(7) | A_BOLD);
            attron(c485);
            mvprintw(line, 57, "%s", st485.c_str());
            attroff(c485);
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(line, 64, "%4.1fK/s", spd485);
            attroff(COLOR_PAIR(6) | A_BOLD);
            attron(c485);
            mvprintw(line, 73, "(%lluE)", (unsigned long long)e485);
            attroff(c485);
            line++;

            // Row 2: CAN Bus
            bool can_on = engine.can_tester.active.load();
            uint64_t c_err = engine.can_tester.err_frames.load();
            double c_tx_fps = engine.can_tester.tx_fps.load();
            double c_rx_fps = engine.can_tester.rx_fps.load();
            CanDirection c_dir = engine.can_tester.dir.load();

            attron(COLOR_PAIR(7) | A_BOLD);
            std::string dir_label = "CAN(0->1):";
            if (c_dir == CanDirection::CAN1_TO_CAN0) dir_label = "CAN(1->0):";
            else if (c_dir == CanDirection::PING_PONG) dir_label = "CAN(0<->1):";
            mvprintw(line, 3, "%-11s", dir_label.c_str());
            attroff(COLOR_PAIR(7) | A_BOLD);

            std::string can_st = can_on ? "[RUN]" : "[IDLE]";
            int can_col = can_on ? ((c_err == 0) ? (COLOR_PAIR(1) | A_BOLD) : (COLOR_PAIR(3) | A_BOLD)) : COLOR_PAIR(7);
            attron(can_col);
            mvprintw(line, 15, "%s", can_st.c_str());
            attroff(can_col);

            attron(COLOR_PAIR(7));
            if (c_dir == CanDirection::CAN0_TO_CAN1) {
                mvprintw(line, 22, "TX(c0):%4.0fF/s", c_tx_fps);
                mvprintw(line, 37, "|");
                mvprintw(line, 39, "RX(c1):%4.0fF/s", c_rx_fps);
            } else if (c_dir == CanDirection::CAN1_TO_CAN0) {
                mvprintw(line, 22, "TX(c1):%4.0fF/s", c_tx_fps);
                mvprintw(line, 37, "|");
                mvprintw(line, 39, "RX(c0):%4.0fF/s", c_rx_fps);
            } else {
                mvprintw(line, 22, "Ping:%4.0fF/s", c_tx_fps);
                mvprintw(line, 37, "|");
                mvprintw(line, 39, "Pong:%4.0fF/s", c_rx_fps);
            }
            attroff(COLOR_PAIR(7));

            int err_col = (c_err == 0) ? (can_on ? (COLOR_PAIR(1) | A_BOLD) : COLOR_PAIR(7)) : (COLOR_PAIR(3) | A_BOLD);
            attron(err_col);
            mvprintw(line, 55, "(%lluE)", (unsigned long long)c_err);
            attroff(err_col);

            std::string can_tag = can_on ? ((c_err == 0) ? "[PASS]" : "[ERR]") : "[IDLE]";
            int tag_col = can_on ? ((c_err == 0) ? (COLOR_PAIR(1) | A_BOLD) : (COLOR_PAIR(3) | A_BOLD)) : COLOR_PAIR(7);
            attron(tag_col);
            mvprintw(line, 73, "%s", can_tag.c_str());
            attroff(tag_col);
            line++;

            // Spacing before Section 4
            line++;

            // ====================================================================
            // SECTION 4: 4-PORT HIGH-SPEED USB STORAGE
            // ====================================================================
            draw_section_banner(line++, w, 12, 6, "[8] 4-PORT USB STORAGE", "sda..sdd Read/Write Stress & Integrity");

            bool usb_on = engine.usb_tester.active.load();
            uint64_t usb_w_mb = engine.usb_tester.get_total_write_mb();
            uint64_t usb_r_mb = engine.usb_tester.get_total_read_mb();
            uint64_t usb_errs = engine.usb_tester.get_total_errors();
            double usb_w_spd = engine.usb_tester.get_total_write_speed();
            double usb_r_spd = engine.usb_tester.get_total_read_speed();

            auto draw_usb_disk_inline = [&](int y, int x, const UsbDiskInfo& d, bool is_first) {
                if (!d.mounted) {
                    if (d.errors.load() > 0) {
                        attron(COLOR_PAIR(3) | A_BOLD);
                        mvprintw(y, x, "%s%-3s: [DISCONNECTED (%lluE)]",
                                 is_first ? "* " : "", d.dev_name.c_str(),
                                 (unsigned long long)d.errors.load());
                        attroff(COLOR_PAIR(3) | A_BOLD);
                    } else {
                        attron(COLOR_PAIR(8));
                        mvprintw(y, x, "%s%-3s: [UNPLUGGED / NO DEV]", is_first ? "* " : "", d.dev_name.c_str());
                        attroff(COLOR_PAIR(8));
                    }
                    return;
                }

                attron(COLOR_PAIR(2) | A_BOLD);
                mvprintw(y, x, "%s%-3s:", is_first ? "* " : "", d.dev_name.c_str());
                attroff(COLOR_PAIR(2) | A_BOLD);

                int px = x + (is_first ? 7 : 5);

                uint64_t w_mb = d.write_bytes.load() / (1024 * 1024);
                uint64_t r_mb = d.read_bytes.load() / (1024 * 1024);
                double wspd = d.write_speed_mb.load();
                double rspd = d.read_speed_mb.load();
                uint64_t derr = d.errors.load();

                if (!usb_on && w_mb == 0 && r_mb == 0) {
                    attron(COLOR_PAIR(1));
                    std::string m = d.model.empty() ? "Ready" : d.model.substr(0, 16);
                    mvprintw(y, px, "Ready (%s)", m.c_str());
                    attroff(COLOR_PAIR(1));
                } else {
                    attron(COLOR_PAIR(7));
                    mvprintw(y, px, "W:%-3lluM", (unsigned long long)w_mb);
                    attroff(COLOR_PAIR(7));

                    attron(COLOR_PAIR(6));
                    mvprintw(y, px + 7, "(%2.0fM)", wspd);
                    attroff(COLOR_PAIR(6));

                    attron(COLOR_PAIR(7));
                    mvprintw(y, px + 14, "R:%-3lluM", (unsigned long long)r_mb);
                    attroff(COLOR_PAIR(7));

                    attron(COLOR_PAIR(6));
                    mvprintw(y, px + 21, "(%2.0fM)", rspd);
                    attroff(COLOR_PAIR(6));

                    int tag_col = (derr == 0) ? (COLOR_PAIR(1) | A_BOLD) : (COLOR_PAIR(3) | A_BOLD);
                    std::string tag = (derr == 0) ? "[OK]" : ("[" + std::to_string(derr) + "E]");
                    attron(tag_col);
                    mvprintw(y, px + 28, "%s", tag.c_str());
                    attroff(tag_col);
                }
            };

            // Row 1: Total USB summary
            attron(COLOR_PAIR(7) | A_BOLD);
            mvprintw(line, 3, "4-Disk Stress:");
            attroff(COLOR_PAIR(7) | A_BOLD);

            std::string usb_st = usb_on ? "[RUNNING]" : "[STANDBY]";
            int usb_col = usb_on ? (COLOR_PAIR(1) | A_BOLD) : COLOR_PAIR(2);
            attron(usb_col);
            mvprintw(line, 18, "%s", usb_st.c_str());
            attroff(usb_col);

            attron(COLOR_PAIR(7));
            mvprintw(line, 28, "W:%-4lluM (%2.0fM/s) | R:%-4lluM (%2.0fM/s)",
                     (unsigned long long)usb_w_mb, usb_w_spd,
                     (unsigned long long)usb_r_mb, usb_r_spd);
            attroff(COLOR_PAIR(7));

            int u_err_col = (usb_errs == 0) ? (usb_on ? (COLOR_PAIR(1) | A_BOLD) : COLOR_PAIR(7)) : (COLOR_PAIR(3) | A_BOLD);
            std::string u_tag = usb_on ? ((usb_errs == 0) ? "[PASS]" : "[ERR]") : "[IDLE]";
            attron(u_err_col);
            mvprintw(line, 73, "%s", u_tag.c_str());
            attroff(u_err_col);
            line++;

            // Row 2 of USB: sda & sdb
            if (engine.usb_tester.disks.size() >= 2) {
                draw_usb_disk_inline(line, 3, *engine.usb_tester.disks[0], true);
                attron(COLOR_PAIR(7));
                mvprintw(line, 40, "|");
                attroff(COLOR_PAIR(7));
                draw_usb_disk_inline(line, 42, *engine.usb_tester.disks[1], false);
                line++;
            }

            // Row 3 of USB: sdc & sdd
            if (engine.usb_tester.disks.size() >= 4) {
                draw_usb_disk_inline(line, 3, *engine.usb_tester.disks[2], true);
                attron(COLOR_PAIR(7));
                mvprintw(line, 40, "|");
                attroff(COLOR_PAIR(7));
                draw_usb_disk_inline(line, 42, *engine.usb_tester.disks[3], false);
                line++;
            }

            // Bottom Footer (Anchored cleanly)
            int key_y = std::max(line, h - 2);
            int status_y = std::max(line + 1, h - 1);

            attron(COLOR_PAIR(2) | A_BOLD);
            mvprintw(key_y, 2, "[Keys] [M]Menu [1..3]Stress [5]Ping [6]232 [7]485 [8]USB [9]CAN [D]Dir [Q]Quit");
            attroff(COLOR_PAIR(2) | A_BOLD);

            attron(COLOR_PAIR(7) | A_BOLD);
            mvprintw(status_y, 2, "Status: %-70s", prompt_status.c_str());
            attroff(COLOR_PAIR(7) | A_BOLD);

            refresh();
        }

        usleep(15000);
    }

    engine.stop_all_stress();
    endwin();

    // 退出时打印清晰的中文报告
    print_summary_report(&engine);

    return 0;
}
