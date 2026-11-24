#include "common.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>

const std::string DEVFREQ_GPU = "/sys/class/devfreq/27800000.gpu";
const std::string DEVFREQ_NPU = "/sys/class/devfreq/27700000.npu";
const std::string DEBUG_RKNPU_LOAD = "/sys/kernel/debug/rknpu/load";
const std::string DEBUG_RKRGA_LOAD = "/sys/kernel/debug/rkrga/load";
const std::string DEBUG_CLK_RGA = "/sys/kernel/debug/clk/clk_core_rga2e_0/clk_rate";
const std::string DEBUG_CLK_VENC = "/sys/kernel/debug/clk/clk_vepu0_core/clk_rate";
const std::string DEBUG_CLK_VDEC = "/sys/kernel/debug/clk/clk_rkvdec_core/clk_rate";

volatile sig_atomic_t g_quit_flag = 0;

void sig_handler(int sig) {
    (void)sig;
    g_quit_flag = 1;
}

uint16_t in_cksum(const void* buf, int len) {
    const auto* p = static_cast<const uint8_t*>(buf);
    uint32_t sum = 0;
    int nleft = len;
    while (nleft > 1) {
        uint16_t word = 0;
        std::memcpy(&word, p, sizeof(word));
        sum += word;
        p += 2;
        nleft -= 2;
    }
    if (nleft == 1) {
        uint16_t last = 0;
        std::memcpy(&last, p, 1);
        sum += last;
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFFu) + (sum >> 16);
    }
    return static_cast<uint16_t>(~sum);
}

uint64_t get_time_us() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

std::string get_time_str() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    struct tm tm_buf{};
    localtime_r(&in_time_t, &tm_buf);
    std::stringstream ss;
    ss << std::put_time(&tm_buf, "%H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

std::string find_shared_lib(const std::vector<std::string>& names) {
    static const char* dirs[] = {
        "/usr/lib/aarch64-linux-gnu/",
        "/lib/aarch64-linux-gnu/",
        "/usr/local/lib/",
        "/usr/lib/",
        "/lib/"
    };
    for (const auto& name : names) {
        for (const char* d : dirs) {
            std::string p = std::string(d) + name;
            if (access(p.c_str(), R_OK) == 0) return p;
        }
    }
    return "";
}

bool is_valid_block_dev_name(const std::string& dev) {
    if (dev.size() < 3 || dev.size() > 8) return false;
    if (dev[0] != 's' || dev[1] != 'd') return false;
    for (size_t i = 2; i < dev.size(); ++i) {
        if (dev[i] < 'a' || dev[i] > 'z') return false;
    }
    return true;
}

bool is_valid_part_path(const std::string& part) {
    if (part.rfind("/dev/sd", 0) != 0 || part.size() < 8 || part.size() > 16) return false;
    size_t i = 7;
    while (i < part.size() && part[i] >= 'a' && part[i] <= 'z') ++i;
    if (i == 7) return false;
    while (i < part.size() && part[i] >= '0' && part[i] <= '9') ++i;
    return i == part.size();
}
