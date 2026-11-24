#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <sys/types.h>
#include <vector>

// 父子进程间共享的轻量统计页（单页 4KB 匿名 memfd，彻底隔离大容量 DDR mmap_lock）
struct DdrSharedStats {
    std::atomic<uint64_t> alloc_mb{0};
    std::atomic<uint64_t> bytes_tested{0};
    std::atomic<uint64_t> passes{0};
    std::atomic<uint64_t> bit_errors{0};
    std::atomic<uint64_t> bw_gb_s_x1000{0};
};

// 独立子进程入口：--internal-ddr-burn <shm_fd> <total_mb> <cores_csv>
int run_internal_ddr_burn(int shm_fd, uint64_t total_mb, const std::string& cores_csv);

// 大容量 DDR 带宽与位翻转完整性校验压测控制器（通过独立子进程执行，杜绝主进程 mmap_lock 争抢）
class DdrStressTester {
public:
    std::atomic<bool> active{false};

    DdrStressTester() = default;
    ~DdrStressTester();

    void start(const std::vector<int>& allowed_cores);
    void stop();
    void reset();

    // 供引擎并行停机路径使用
    pid_t get_pid() const;
    void notify_reaped();
    std::string check_health();

    uint64_t get_alloc_mb() const;
    uint64_t get_bytes_tested() const;
    uint64_t get_passes() const;
    uint64_t get_bit_errors() const;
    double get_bw_gb_s() const;
    std::string get_status_detail() const;

private:
    mutable std::mutex state_mtx;
    mutable std::mutex detail_mtx;
    std::string status_detail;

    int shm_fd{-1};
    DdrSharedStats* shm{nullptr};
    pid_t child_pid{0};

    uint64_t last_alloc_mb{0};
    uint64_t saved_bytes{0};
    uint64_t saved_passes{0};
    uint64_t saved_errors{0};
    uint64_t base_bytes{0};
    uint64_t base_passes{0};
    uint64_t base_errors{0};

    bool ensure_shm_locked();
    void snapshot_and_clear_locked();
    void set_detail(const std::string& msg);
    static uint64_t read_mem_available_mb();
};
