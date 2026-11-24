#include "ddr_tester.hpp"
#include "common.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

static_assert(std::atomic<uint64_t>::is_always_lock_free, "DdrSharedStats requires lock-free 64-bit atomics");

namespace {

int create_anon_shm_fd(size_t size) {
    int fd = -1;
#ifdef SYS_memfd_create
    fd = static_cast<int>(syscall(SYS_memfd_create, "rk_ddr_shm", 0U));
#endif
    if (fd < 0) {
        char tmpl[] = "/dev/shm/rk_ddr_XXXXXX";
        fd = mkstemp(tmpl);
        if (fd >= 0) unlink(tmpl);
    }
    if (fd < 0) return -1;
    // 父进程保持 FD_CLOEXEC，避免泄露给其他子进程；仅在 DDR 子进程 execl 前清除
    int flags = fcntl(fd, F_GETFD);
    if (flags >= 0) fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
    if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

std::vector<int> parse_cores_csv(const std::string& csv) {
    std::vector<int> cores;
    const char* p = csv.c_str();
    while (*p) {
        char* endp = nullptr;
        long v = std::strtol(p, &endp, 10);
        if (endp && endp != p && v >= 0 && v < 128) {
            cores.push_back(static_cast<int>(v));
        }
        if (!endp || *endp == '\0') break;
        p = (*endp == ',') ? (endp + 1) : endp;
    }
    return cores;
}

} // namespace

int run_internal_ddr_burn(int shm_fd, uint64_t total_mb, const std::string& cores_csv) {
    if (shm_fd < 0 || total_mb < 64) return 1;
    void* mapped = mmap(nullptr, sizeof(DdrSharedStats), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (mapped == MAP_FAILED) return 2;
    auto* shm = static_cast<DdrSharedStats*>(mapped);

    std::vector<int> allowed_cores = parse_cores_csv(cores_csv);
    size_t n_workers = 4;
    if (!allowed_cores.empty() && allowed_cores.size() < n_workers) {
        n_workers = allowed_cores.size();
    }
    uint64_t per_mb = std::max<uint64_t>(32, (total_mb / n_workers) & ~3ULL);
    uint64_t actual_total_mb = per_mb * n_workers;
    size_t region_bytes = static_cast<size_t>(per_mb) * 1024ULL * 1024ULL;
    shm->alloc_mb.store(actual_total_mb, std::memory_order_relaxed);

    std::atomic<uint64_t> window_bytes{0};
    std::vector<std::thread> workers;
    workers.reserve(n_workers);

    auto worker_fn = [&](size_t worker_idx, int pin_core) {
        if (pin_core >= 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(pin_core, &cpuset);
            pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
        }

        // 子进程内同样使用 SCHED_IDLE，避免饿死 GPU/NPU/RGA/MPP 与工业总线/网口线程
        struct sched_param sp{};
        sp.sched_priority = 0;
        if (pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp) != 0) {
            setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 19);
        }

        void* mem = mmap(nullptr, region_bytes, PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        if (mem == MAP_FAILED) return;
#ifdef MADV_HUGEPAGE
        madvise(mem, region_bytes, MADV_HUGEPAGE);
#endif

        auto* base = static_cast<uint64_t*>(mem);
        const size_t total_words = region_bytes / sizeof(uint64_t);
        // 1MB 分片检查退出标志，兼顾满带宽与毫秒级响应退出
        constexpr size_t kChunkWords = (1024 * 1024) / sizeof(uint64_t);
        uint64_t pass_idx = 0;

        while (!g_quit_flag) {
            uint64_t phase = pass_idx & 3ULL;
            uint64_t seed = 0x9E3779B97F4A7C15ULL * (pass_idx + 1ULL) +
                            0xBF58476D1CE4E5B9ULL * (static_cast<uint64_t>(worker_idx) + 1ULL);
            uint64_t xor_mask = 0;
            uint64_t addr_mul = 0;
            if (phase == 0) {
                xor_mask = seed;
                addr_mul = 0x100000001B3ULL;
            } else if (phase == 1) {
                xor_mask = ~seed;
                addr_mul = 0x100000001B3ULL;
            } else if (phase == 2) {
                xor_mask = 0x55AA55AA55AA55AAULL ^ (seed & 0xFFFFULL);
                addr_mul = 0x0001000100010001ULL;
            } else {
                xor_mask = 0xAA55AA55AA55AA55ULL ^ (seed & 0xFFFFULL);
                addr_mul = 0x0001000100010001ULL;
            }

            // Phase A: 大容量顺序写刷入物理 LPDDR 颗粒
            for (size_t off = 0; off < total_words && !g_quit_flag; off += kChunkWords) {
                size_t end = std::min(total_words, off + kChunkWords);
                uint64_t* p = base + off;
                size_t cnt = end - off;
#if defined(__aarch64__) || defined(__ARM_NEON)
                uint64x2_t step2 = vdupq_n_u64(addr_mul * 2ULL);
                uint64x2_t step4 = vdupq_n_u64(addr_mul * 4ULL);
                uint64x2_t vx = vdupq_n_u64(xor_mask);
                uint64_t base_a = static_cast<uint64_t>(off) * addr_mul;
                uint64x2_t a0 = {base_a, base_a + addr_mul};
                uint64x2_t a1 = vaddq_u64(a0, step2);
                size_t j = 0;
                for (; j + 4 <= cnt; j += 4) {
                    vst1q_u64(p + j,     veorq_u64(a0, vx));
                    vst1q_u64(p + j + 2, veorq_u64(a1, vx));
                    a0 = vaddq_u64(a0, step4);
                    a1 = vaddq_u64(a1, step4);
                }
                for (; j < cnt; ++j) {
                    p[j] = (static_cast<uint64_t>(off + j) * addr_mul) ^ xor_mask;
                }
#else
                for (size_t j = 0; j < cnt; ++j) {
                    p[j] = (static_cast<uint64_t>(off + j) * addr_mul) ^ xor_mask;
                }
#endif
                uint64_t chunk_b = static_cast<uint64_t>(cnt) * sizeof(uint64_t);
                window_bytes.fetch_add(chunk_b, std::memory_order_relaxed);
                shm->bytes_tested.fetch_add(chunk_b, std::memory_order_relaxed);
            }

            if (g_quit_flag) break;
            __sync_synchronize();

            // Phase B: 从物理 LPDDR 读回并逐 64-bit 向量完整性校验
            for (size_t off = 0; off < total_words && !g_quit_flag; off += kChunkWords) {
                size_t end = std::min(total_words, off + kChunkWords);
                const uint64_t* p = base + off;
                size_t cnt = end - off;
                uint64_t errs = 0;
#if defined(__aarch64__) || defined(__ARM_NEON)
                uint64x2_t step2 = vdupq_n_u64(addr_mul * 2ULL);
                uint64x2_t step4 = vdupq_n_u64(addr_mul * 4ULL);
                uint64x2_t vx = vdupq_n_u64(xor_mask);
                uint64_t base_a = static_cast<uint64_t>(off) * addr_mul;
                uint64x2_t a0 = {base_a, base_a + addr_mul};
                uint64x2_t a1 = vaddq_u64(a0, step2);
                uint64x2_t diff_acc = vdupq_n_u64(0);
                size_t j = 0;
                for (; j + 4 <= cnt; j += 4) {
                    uint64x2_t r0 = vld1q_u64(p + j);
                    uint64x2_t r1 = vld1q_u64(p + j + 2);
                    uint64x2_t e0 = veorq_u64(a0, vx);
                    uint64x2_t e1 = veorq_u64(a1, vx);
                    diff_acc = vorrq_u64(diff_acc, vorrq_u64(veorq_u64(r0, e0), veorq_u64(r1, e1)));
                    a0 = vaddq_u64(a0, step4);
                    a1 = vaddq_u64(a1, step4);
                }
                if ((vgetq_lane_u64(diff_acc, 0) | vgetq_lane_u64(diff_acc, 1)) != 0) {
                    for (size_t k = 0; k < j; ++k) {
                        uint64_t exp = (static_cast<uint64_t>(off + k) * addr_mul) ^ xor_mask;
                        if (p[k] != exp) ++errs;
                    }
                }
                for (; j < cnt; ++j) {
                    uint64_t exp = (static_cast<uint64_t>(off + j) * addr_mul) ^ xor_mask;
                    if (p[j] != exp) ++errs;
                }
#else
                for (size_t j = 0; j < cnt; ++j) {
                    uint64_t exp = (static_cast<uint64_t>(off + j) * addr_mul) ^ xor_mask;
                    if (p[j] != exp) ++errs;
                }
#endif
                if (errs > 0) shm->bit_errors.fetch_add(errs, std::memory_order_relaxed);
                uint64_t chunk_b = static_cast<uint64_t>(cnt) * sizeof(uint64_t);
                window_bytes.fetch_add(chunk_b, std::memory_order_relaxed);
                shm->bytes_tested.fetch_add(chunk_b, std::memory_order_relaxed);
            }

            if (!g_quit_flag) {
                shm->passes.fetch_add(1, std::memory_order_relaxed);
                ++pass_idx;
            }
        }
        // 退出时不在此逐线程 munmap，交由进程退出时内核 exit_mmap() 一次性无锁批量回收，避免 4 线程争抢 mmap_lock
    };

    for (size_t i = 0; i < n_workers; ++i) {
        int core = -1;
        if (!allowed_cores.empty()) {
            size_t pick = (i * allowed_cores.size()) / n_workers;
            core = allowed_cores[pick % allowed_cores.size()];
        }
        workers.emplace_back(worker_fn, i, core);
    }

    // 主线程每 500ms 刷新一次实时带宽，SIGTERM 通过无 SA_RESTART 信号瞬间打断休眠
    auto last = std::chrono::steady_clock::now();
    while (!g_quit_flag) {
        for (int s = 0; s < 25 && !g_quit_flag; ++s) {
            usleep(20000);
        }
        if (g_quit_flag) break;
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count() / 1000.0;
        last = now;
        uint64_t b = window_bytes.exchange(0, std::memory_order_relaxed);
        if (dt > 0.05) {
            double gb_s = static_cast<double>(b) / (1024.0 * 1024.0 * 1024.0) / dt;
            shm->bw_gb_s_x1000.store(static_cast<uint64_t>(gb_s * 1000.0 + 0.5), std::memory_order_relaxed);
        }
    }

    // 提权回 SCHED_OTHER 防止高负载下被饿死，快速回收全部工作线程后进程整体退出释放内存
    struct sched_param sp{};
    sp.sched_priority = 0;
    for (auto& t : workers) {
        pthread_setschedparam(t.native_handle(), SCHED_OTHER, &sp);
    }
    for (auto& t : workers) {
        if (t.joinable()) t.join();
    }
    shm->bw_gb_s_x1000.store(0, std::memory_order_relaxed);
    return 0;
}

DdrStressTester::~DdrStressTester() {
    stop();
    std::lock_guard<std::mutex> lk(state_mtx);
    if (shm && shm != MAP_FAILED) {
        munmap(shm, sizeof(DdrSharedStats));
        shm = nullptr;
    }
    if (shm_fd >= 0) {
        close(shm_fd);
        shm_fd = -1;
    }
}

void DdrStressTester::set_detail(const std::string& msg) {
    std::lock_guard<std::mutex> lk(detail_mtx);
    status_detail = msg;
}

std::string DdrStressTester::get_status_detail() const {
    std::lock_guard<std::mutex> lk(detail_mtx);
    return status_detail;
}

uint64_t DdrStressTester::read_mem_available_mb() {
    FILE* f = std::fopen("/proc/meminfo", "r");
    if (!f) return 0;
    long long avail_kb = 0;
    char line[128];
    while (std::fgets(line, sizeof(line), f)) {
        if (std::strncmp(line, "MemAvailable:", 13) == 0) {
            std::sscanf(line + 13, "%lld", &avail_kb);
            break;
        }
    }
    std::fclose(f);
    return (avail_kb > 0) ? static_cast<uint64_t>(avail_kb / 1024) : 0;
}

bool DdrStressTester::ensure_shm_locked() {
    if (shm && shm_fd >= 0) return true;
    shm_fd = create_anon_shm_fd(sizeof(DdrSharedStats));
    if (shm_fd < 0) return false;
    void* p = mmap(nullptr, sizeof(DdrSharedStats), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (p == MAP_FAILED) {
        close(shm_fd);
        shm_fd = -1;
        return false;
    }
    shm = new (p) DdrSharedStats();
    return true;
}

void DdrStressTester::snapshot_and_clear_locked() {
    if (shm) {
        uint64_t cur_b = shm->bytes_tested.load(std::memory_order_relaxed);
        uint64_t cur_p = shm->passes.load(std::memory_order_relaxed);
        uint64_t cur_e = shm->bit_errors.load(std::memory_order_relaxed);
        if (cur_b >= base_bytes) saved_bytes += (cur_b - base_bytes);
        if (cur_p >= base_passes) saved_passes += (cur_p - base_passes);
        if (cur_e >= base_errors) saved_errors += (cur_e - base_errors);
        uint64_t am = shm->alloc_mb.load(std::memory_order_relaxed);
        if (am > 0) last_alloc_mb = am;
        shm->bw_gb_s_x1000.store(0, std::memory_order_relaxed);
    }
    base_bytes = 0;
    base_passes = 0;
    base_errors = 0;
    child_pid = 0;
    active.store(false);
}

void DdrStressTester::start(const std::vector<int>& allowed_cores) {
    std::lock_guard<std::mutex> lk(state_mtx);
    if (active.load()) return;

    uint64_t avail_mb = read_mem_available_mb();
    // 严格保留至少 768MB 系统与 GPU/NPU/RGA/MPP DMA 安全余量，杜绝 OOM
    if (avail_mb < 900) {
        set_detail("Low MemAvailable (<900MB)");
        return;
    }
    uint64_t usable_mb = avail_mb - 768;
    uint64_t target_mb = std::min<uint64_t>(2048, std::max<uint64_t>(128, std::min(avail_mb / 2, usable_mb)));

    size_t n_workers = 4;
    if (!allowed_cores.empty() && allowed_cores.size() < n_workers) {
        n_workers = allowed_cores.size();
    }
    uint64_t per_mb = std::max<uint64_t>(32, (target_mb / n_workers) & ~3ULL);
    uint64_t total_mb = per_mb * n_workers;

    if (!ensure_shm_locked()) {
        set_detail("shm alloc failed");
        return;
    }

    shm->alloc_mb.store(total_mb, std::memory_order_relaxed);
    shm->bytes_tested.store(0, std::memory_order_relaxed);
    shm->passes.store(0, std::memory_order_relaxed);
    shm->bit_errors.store(0, std::memory_order_relaxed);
    shm->bw_gb_s_x1000.store(0, std::memory_order_relaxed);
    base_bytes = 0;
    base_passes = 0;
    base_errors = 0;
    last_alloc_mb = total_mb;

    std::string cores_csv;
    for (size_t i = 0; i < allowed_cores.size(); ++i) {
        if (i > 0) cores_csv.push_back(',');
        cores_csv += std::to_string(allowed_cores[i]);
    }
    std::string fd_arg = std::to_string(shm_fd);
    std::string mb_arg = std::to_string(total_mb);

    pid_t parent_pid = getpid();
    pid_t pid = fork();
    if (pid == 0) {
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        if (getppid() != parent_pid) _exit(0);
        setsid();
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        int fd_flags = fcntl(shm_fd, F_GETFD);
        if (fd_flags >= 0) fcntl(shm_fd, F_SETFD, fd_flags & ~FD_CLOEXEC);
        execl("/proc/self/exe", "rk_tester", "--internal-ddr-burn",
              fd_arg.c_str(), mb_arg.c_str(), cores_csv.c_str(), nullptr);
        _exit(127);
    }

    if (pid > 0) {
        child_pid = pid;
        set_detail(std::to_string(total_mb) + "MB (" + std::to_string(n_workers) + "T)");
        active.store(true);
    } else {
        set_detail("DDR stress fork failed");
    }
}

void DdrStressTester::stop() {
    pid_t pid = 0;
    {
        std::lock_guard<std::mutex> lk(state_mtx);
        if (!active.load() && child_pid <= 0) return;
        pid = child_pid;
    }
    if (pid > 0) {
        kill(-pid, SIGTERM);
        bool reaped = false;
        for (int i = 0; i < 40; ++i) {
            if (waitpid(pid, nullptr, WNOHANG) == pid) {
                reaped = true;
                break;
            }
            usleep(5000);
        }
        if (!reaped) {
            kill(-pid, SIGKILL);
            waitpid(pid, nullptr, 0);
        }
    }
    std::lock_guard<std::mutex> lk(state_mtx);
    snapshot_and_clear_locked();
}

pid_t DdrStressTester::get_pid() const {
    std::lock_guard<std::mutex> lk(state_mtx);
    return child_pid;
}

void DdrStressTester::notify_reaped() {
    std::lock_guard<std::mutex> lk(state_mtx);
    snapshot_and_clear_locked();
}

std::string DdrStressTester::check_health() {
    std::lock_guard<std::mutex> lk(state_mtx);
    if (active.load() && child_pid > 0) {
        int st = 0;
        if (waitpid(child_pid, &st, WNOHANG) == child_pid) {
            snapshot_and_clear_locked();
            return "DDR stress exited unexpectedly (check available memory).";
        }
    }
    return "";
}

void DdrStressTester::reset() {
    std::lock_guard<std::mutex> lk(state_mtx);
    saved_bytes = 0;
    saved_passes = 0;
    saved_errors = 0;
    if (active.load() && shm) {
        base_bytes = shm->bytes_tested.load(std::memory_order_relaxed);
        base_passes = shm->passes.load(std::memory_order_relaxed);
        base_errors = shm->bit_errors.load(std::memory_order_relaxed);
    } else {
        base_bytes = 0;
        base_passes = 0;
        base_errors = 0;
        last_alloc_mb = 0;
        if (shm) {
            shm->alloc_mb.store(0, std::memory_order_relaxed);
            shm->bytes_tested.store(0, std::memory_order_relaxed);
            shm->passes.store(0, std::memory_order_relaxed);
            shm->bit_errors.store(0, std::memory_order_relaxed);
            shm->bw_gb_s_x1000.store(0, std::memory_order_relaxed);
        }
    }
}

uint64_t DdrStressTester::get_alloc_mb() const {
    std::lock_guard<std::mutex> lk(state_mtx);
    if (active.load() && shm) {
        uint64_t v = shm->alloc_mb.load(std::memory_order_relaxed);
        return (v > 0) ? v : last_alloc_mb;
    }
    return last_alloc_mb;
}

uint64_t DdrStressTester::get_bytes_tested() const {
    std::lock_guard<std::mutex> lk(state_mtx);
    uint64_t tot = saved_bytes;
    if (active.load() && shm) {
        uint64_t cur = shm->bytes_tested.load(std::memory_order_relaxed);
        if (cur >= base_bytes) tot += (cur - base_bytes);
    }
    return tot;
}

uint64_t DdrStressTester::get_passes() const {
    std::lock_guard<std::mutex> lk(state_mtx);
    uint64_t tot = saved_passes;
    if (active.load() && shm) {
        uint64_t cur = shm->passes.load(std::memory_order_relaxed);
        if (cur >= base_passes) tot += (cur - base_passes);
    }
    return tot;
}

uint64_t DdrStressTester::get_bit_errors() const {
    std::lock_guard<std::mutex> lk(state_mtx);
    uint64_t tot = saved_errors;
    if (active.load() && shm) {
        uint64_t cur = shm->bit_errors.load(std::memory_order_relaxed);
        if (cur >= base_errors) tot += (cur - base_errors);
    }
    return tot;
}

double DdrStressTester::get_bw_gb_s() const {
    std::lock_guard<std::mutex> lk(state_mtx);
    if (active.load() && shm) {
        return static_cast<double>(shm->bw_gb_s_x1000.load(std::memory_order_relaxed)) / 1000.0;
    }
    return 0.0;
}
