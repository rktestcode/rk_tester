#pragma once

#include <csignal>
#include <cstdint>
#include <string>
#include <vector>

// 硬件路径定义
extern const std::string DEVFREQ_GPU;
extern const std::string DEVFREQ_NPU;
extern const std::string DEBUG_RKNPU_LOAD;
extern const std::string DEBUG_RKRGA_LOAD;
extern const std::string DEBUG_CLK_RGA;
extern const std::string DEBUG_CLK_VENC;
extern const std::string DEBUG_CLK_VDEC;

// 异步信号安全的退出标志
extern volatile sig_atomic_t g_quit_flag;

void sig_handler(int sig);

enum class ChannelState {
    STANDBY = 0,
    RUNNING = 1,
    PORT_ERROR = 2
};

// 严格别名安全的 RFC 1071 校验和计算
uint16_t in_cksum(const void* buf, int len);

uint64_t get_time_us();
std::string get_time_str();

std::string find_shared_lib(const std::vector<std::string>& names);

bool is_valid_block_dev_name(const std::string& dev);
bool is_valid_part_path(const std::string& part);
