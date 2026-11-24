#include "soc_engine.hpp"
#include "common.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <pthread.h>
#include <sched.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>
#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace {

std::string read_text_token(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return "";
    char buf[128]{};
    std::string res;
    if (std::fscanf(f, "%127s", buf) == 1) res = buf;
    std::fclose(f);
    return res;
}

void write_text_value(const std::string& path, const std::string& val) {
    std::ofstream f(path);
    if (f.is_open()) f << val;
}

long long read_ll_file(const std::string& path, long long def_val = 0) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return def_val;
    long long v = def_val;
    if (std::fscanf(f, "%lld", &v) != 1) v = def_val;
    std::fclose(f);
    return v;
}

int read_debugfs_clk_mhz(const std::string& path) {
    long long hz = read_ll_file(path, 0);
    return (hz > 0) ? static_cast<int>(hz / 1000000) : 0;
}

pid_t spawn_internal_worker(const char* flag) {
    pid_t parent_pid = getpid();
    pid_t pid = fork();
    if (pid == 0) {
        // 绑定父进程退出信号：即使父进程被 SIGKILL 或 SSH 断线，子进程也会立刻收到 SIGTERM 自动清理，杜绝孤儿烤机进程
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        if (getppid() != parent_pid) _exit(0);
        setsid();
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        execl("/proc/self/exe", "rk_tester", flag, nullptr);
        _exit(127);
    }
    return pid;
}

void read_mpp_irq_counts(uint64_t& enc_irqs, uint64_t& dec_irqs) {
    enc_irqs = 0;
    dec_irqs = 0;
    FILE* f = std::fopen("/proc/interrupts", "r");
    if (!f) return;
    char line[512];
    while (std::fgets(line, sizeof(line), f)) {
        bool is_enc = (std::strstr(line, "rkvenc") != nullptr);
        bool is_dec = (std::strstr(line, "rkvdec") != nullptr);
        if (!is_enc && !is_dec) continue;
        char* p = std::strchr(line, ':');
        if (!p) continue;
        ++p;
        uint64_t sum = 0;
        while (*p) {
            while (*p == ' ' || *p == '\t') ++p;
            if (*p < '0' || *p > '9') break;
            char* endp = nullptr;
            unsigned long long v = std::strtoull(p, &endp, 10);
            if (!endp || endp == p) break;
            if (*endp != ' ' && *endp != '\t' && *endp != '\n' && *endp != '\0') break;
            sum += static_cast<uint64_t>(v);
            p = endp;
        }
        if (is_enc) enc_irqs += sum;
        if (is_dec) dec_irqs += sum;
    }
    std::fclose(f);
}

void stop_child_process(pid_t& pid) {
    if (pid <= 0) return;
    kill(-pid, SIGTERM);
    bool reaped = false;
    for (int i = 0; i < 40; ++i) {
        if (waitpid(pid, nullptr, WNOHANG) == pid) {
            reaped = true;
            break;
        }
        usleep(10000);
    }
    if (!reaped) {
        kill(-pid, SIGKILL);
        waitpid(pid, nullptr, 0);
    }
    pid = 0;
}

} // namespace

RkTesterEngine::RkTesterEngine() {
    num_cores = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    if (num_cores <= 0) num_cores = 8;
    target_cpu_cores = num_cores;
    prev_cores.resize(num_cores);
    smooth_cores.resize(num_cores, 0.0);
    rebuild_cpu_burn_list();

    init_governor_state();
    init_thermals();
    init_cooling_devices();
    read_mpp_irq_counts(mpp_base_enc_irqs, mpp_base_dec_irqs);
    prev_mpp_enc_irqs = mpp_base_enc_irqs;
    prev_mpp_dec_irqs = mpp_base_dec_irqs;
    prev_mpp_sample_us = get_time_us();
    mpp_irq_initialized = true;
}

RkTesterEngine::~RkTesterEngine() {
    stop_all_stress();
}

void RkTesterEngine::toggle_rs232() {
    if (rs232.active.load()) rs232.stop();
    else rs232.start();
}

void RkTesterEngine::toggle_rs485() {
    if (rs485.active.load()) rs485.stop();
    else rs485.start();
}

void RkTesterEngine::toggle_usb_stress() {
    if (usb_tester.active.load()) usb_tester.stop();
    else usb_tester.start();
}

void RkTesterEngine::toggle_can() {
    if (can_tester.active.load()) can_tester.stop();
    else can_tester.start();
}

EmcSnapshot RkTesterEngine::get_emc_snapshot() const {
    return net_emc.get_snapshot();
}

void RkTesterEngine::set_port_enabled(size_t idx, bool en) {
    net_emc.set_port_enabled(idx, en);
}

bool RkTesterEngine::get_port_enabled(size_t idx) const {
    return net_emc.get_port_enabled(idx);
}

void RkTesterEngine::log_emc_event(const std::string& level, const std::string& msg, int color) {
    net_emc.log_event(level, msg, color);
}

void RkTesterEngine::setup_default_network() {
    net_emc.setup_default_network();
}

void RkTesterEngine::reset_all_stats() {
    ddr_tester.reset();
    net_emc.reset_stats();
    rs232.reset();
    rs485.reset();
    can_tester.reset();
    usb_tester.reset();
    read_mpp_irq_counts(mpp_base_enc_irqs, mpp_base_dec_irqs);
    prev_mpp_enc_irqs = mpp_base_enc_irqs;
    prev_mpp_dec_irqs = mpp_base_dec_irqs;
    prev_mpp_sample_us = get_time_us();
}

void RkTesterEngine::set_keep_isolcpus(bool keep) {
    keep_isolcpus = keep;
    rebuild_cpu_burn_list();
}

int RkTesterEngine::get_active_cpu_burn_cores() const {
    return static_cast<int>(cpu_burn_cores.size());
}

void RkTesterEngine::toggle_cpu_stress() {
    if (cpu_stress.load()) {
        cpu_stress.store(false);
        // 停止时将 SCHED_IDLE 线程提权回 SCHED_OTHER，避免被其他普通优先级进程饿死导致 join 卡顿
        struct sched_param sp{};
        sp.sched_priority = 0;
        for (auto& t : cpu_threads) {
            if (t.joinable()) {
                pthread_setschedparam(t.native_handle(), SCHED_OTHER, &sp);
            }
        }
        for (auto& t : cpu_threads) {
            if (t.joinable()) t.join();
        }
        cpu_threads.clear();
    } else {
        cpu_stress.store(true);
        if (cpu_burn_cores.empty()) rebuild_cpu_burn_list();
        for (int core_id : cpu_burn_cores) {
            int i = core_id;
            cpu_threads.emplace_back([this, i]() {
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(i % num_cores, &cpuset);
                pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

                // 将纯 CPU 烤机线程设为 SCHED_IDLE（或最低 nice=19），确保吃满 100% 空闲算力的同时
                // 绝不饿死 NPU(950MHz) / RKMPP / RGA / GPU 硬件提交线程与 TUI/串口/网口监控线程
                struct sched_param sp{};
                sp.sched_priority = 0;
                if (pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp) != 0) {
                    setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 19);
                }

                // 每核独立 4MB (2x 2MB) DDR 流缓冲区，跨出 L2 Cache 同步压榨 DDR 控制器与 DRAM 发热
                constexpr size_t kElems = 512 * 1024; // 512K floats = 2MB per buffer
                std::vector<float> buf_a(kElems);
                std::vector<float> buf_b(kElems);
                for (size_t idx = 0; idx < kElems; ++idx) {
                    buf_a[idx] = 0.75f + static_cast<float>((idx + static_cast<size_t>(i * 17)) & 255) * 0.0023f;
                    buf_b[idx] = 0.55f + static_cast<float>((idx * 3 + static_cast<size_t>(i * 31)) & 255) * 0.0019f;
                }

                volatile float sink = 0.0f;
#if defined(__aarch64__) || defined(__ARM_NEON)
                // 12 路独立 128-bit ARM NEON FMA 累加器，完全消除 RAW 流水线停顿，满发射压榨 A72/A53 向量单元
                float32x4_t v0  = {1.123f, 0.877f, 1.234f, 0.766f};
                float32x4_t v1  = {0.912f, 1.088f, 0.834f, 1.166f};
                float32x4_t v2  = {1.198f, 0.802f, 1.045f, 0.955f};
                float32x4_t v3  = {0.777f, 1.223f, 0.961f, 1.039f};
                float32x4_t v4  = {1.141f, 0.859f, 1.271f, 0.729f};
                float32x4_t v5  = {0.888f, 1.112f, 0.793f, 1.207f};
                float32x4_t v6  = {1.251f, 0.749f, 1.067f, 0.933f};
                float32x4_t v7  = {0.845f, 1.155f, 0.923f, 1.077f};
                float32x4_t v8  = {1.179f, 0.821f, 1.109f, 0.891f};
                float32x4_t v9  = {0.937f, 1.063f, 0.812f, 1.188f};
                float32x4_t v10 = {1.213f, 0.787f, 1.031f, 0.969f};
                float32x4_t v11 = {0.869f, 1.131f, 0.947f, 1.053f};

                const float32x4_t k_mul = vdupq_n_f32(0.751953125f);
                const float32x4_t k_add = {0.3712345f, 0.4198765f, 0.3356789f, 0.4512341f};
                const float32x4_t k_sub = {1.8734567f, 1.7923456f, 1.9145678f, 1.7456789f};
                const float32x4_t k_s_mul = vdupq_n_f32(0.625f);
                const float32x4_t k_s_add = vdupq_n_f32(0.375f);

                size_t stream_pos = 0;
                while (this->cpu_stress.load(std::memory_order_relaxed)) {
                    // Phase 1: 高位翻转率 12-way NEON FMA 满发射算力烤机（缩短单轮周期兼顾满负载与毫秒级退出响应）
                    for (int k = 0; k < 4000; ++k) {
                        v0  = vfmaq_f32(k_add, v0,  k_mul);
                        v1  = vfmaq_f32(k_add, v1,  k_mul);
                        v2  = vfmaq_f32(k_add, v2,  k_mul);
                        v3  = vfmaq_f32(k_add, v3,  k_mul);
                        v4  = vfmaq_f32(k_add, v4,  k_mul);
                        v5  = vfmaq_f32(k_add, v5,  k_mul);
                        v6  = vfmaq_f32(k_add, v6,  k_mul);
                        v7  = vfmaq_f32(k_add, v7,  k_mul);
                        v8  = vfmaq_f32(k_add, v8,  k_mul);
                        v9  = vfmaq_f32(k_add, v9,  k_mul);
                        v10 = vfmaq_f32(k_add, v10, k_mul);
                        v11 = vfmaq_f32(k_add, v11, k_mul);

                        v0  = vfmsq_f32(k_sub, v0,  k_mul);
                        v1  = vfmsq_f32(k_sub, v1,  k_mul);
                        v2  = vfmsq_f32(k_sub, v2,  k_mul);
                        v3  = vfmsq_f32(k_sub, v3,  k_mul);
                        v4  = vfmsq_f32(k_sub, v4,  k_mul);
                        v5  = vfmsq_f32(k_sub, v5,  k_mul);
                        v6  = vfmsq_f32(k_sub, v6,  k_mul);
                        v7  = vfmsq_f32(k_sub, v7,  k_mul);
                        v8  = vfmsq_f32(k_sub, v8,  k_mul);
                        v9  = vfmsq_f32(k_sub, v9,  k_mul);
                        v10 = vfmsq_f32(k_sub, v10, k_mul);
                        v11 = vfmsq_f32(k_sub, v11, k_mul);
                    }
                    if (!this->cpu_stress.load(std::memory_order_relaxed)) break;

                    // Phase 2: NEON 向量流式读写 (L1/L2/DDR 带宽压测)
                    constexpr size_t kChunk = 32768; // 128KB per pass, sweeps full 4MB
                    float* pa = buf_a.data() + stream_pos;
                    const float* pb = buf_b.data() + stream_pos;
                    for (size_t j = 0; j < kChunk; j += 16) {
                        float32x4_t a0 = vld1q_f32(pa + j);
                        float32x4_t a1 = vld1q_f32(pa + j + 4);
                        float32x4_t a2 = vld1q_f32(pa + j + 8);
                        float32x4_t a3 = vld1q_f32(pa + j + 12);
                        float32x4_t b0 = vld1q_f32(pb + j);
                        float32x4_t b1 = vld1q_f32(pb + j + 4);
                        float32x4_t b2 = vld1q_f32(pb + j + 8);
                        float32x4_t b3 = vld1q_f32(pb + j + 12);
                        vst1q_f32(pa + j,      vfmaq_f32(k_s_add, vfmaq_f32(a0, b0, k_s_mul), k_s_mul));
                        vst1q_f32(pa + j + 4,  vfmaq_f32(k_s_add, vfmaq_f32(a1, b1, k_s_mul), k_s_mul));
                        vst1q_f32(pa + j + 8,  vfmaq_f32(k_s_add, vfmaq_f32(a2, b2, k_s_mul), k_s_mul));
                        vst1q_f32(pa + j + 12, vfmaq_f32(k_s_add, vfmaq_f32(a3, b3, k_s_mul), k_s_mul));
                    }
                    stream_pos = (stream_pos + kChunk) % kElems;

                    float32x4_t acc = vaddq_f32(vaddq_f32(v0, v1), vaddq_f32(v2, v3));
                    acc = vaddq_f32(acc, vaddq_f32(vaddq_f32(v4, v5), vaddq_f32(v6, v7)));
                    acc = vaddq_f32(acc, vaddq_f32(vaddq_f32(v8, v9), vaddq_f32(v10, v11)));
                    sink = vgetq_lane_f32(acc, 0) + vgetq_lane_f32(acc, 1) + pa[0];
                }
#else
                float acc[8] = {1.12f, 0.88f, 1.23f, 0.77f, 1.15f, 0.85f, 1.19f, 0.81f};
                size_t stream_pos = 0;
                while (this->cpu_stress.load(std::memory_order_relaxed)) {
                    for (int k = 0; k < 4000; ++k) {
                        for (int u = 0; u < 8; ++u) {
                            acc[u] = std::fmaf(acc[u], 0.751953125f, 0.3712345f);
                            acc[u] = 1.8734567f - acc[u] * 0.751953125f;
                        }
                    }
                    if (!this->cpu_stress.load(std::memory_order_relaxed)) break;
                    constexpr size_t kChunk = 16384;
                    for (size_t j = 0; j < kChunk; ++j) {
                        size_t p = (stream_pos + j) % kElems;
                        buf_a[p] = std::fmaf(buf_a[p], 0.625f, buf_b[p] * 0.375f);
                    }
                    stream_pos = (stream_pos + kChunk) % kElems;
                    sink = acc[0] + acc[1] + acc[2] + acc[3] + buf_a[stream_pos];
                }
#endif
                (void)sink;
            });
        }
    }
}

std::string RkTesterEngine::toggle_gpu_stress() {
    if (gpu_stress.load()) {
        stop_child_process(gpu_pid);
        gpu_stress.store(false);
        if (gpu_gov_boosted) {
            write_text_value(DEVFREQ_GPU + "/governor", saved_gpu_gov);
            gpu_gov_boosted = false;
        }
        return "GPU stress STOPPED.";
    }

    // 内置 Mesa/Panfrost 无头 GBM + EGL + GLES 离屏着色器压测，直接满频运行
    bool has_drm_gles = (!find_shared_lib({"libEGL.so.1"}).empty() &&
                         !find_shared_lib({"libGLESv2.so.2"}).empty() &&
                         !find_shared_lib({"libgbm.so.1"}).empty() &&
                         (access("/dev/dri/by-path/platform-27800000.gpu-render", R_OK | W_OK) == 0 ||
                          access("/dev/dri/renderD129", R_OK | W_OK) == 0 ||
                          access("/dev/dri/renderD128", R_OK | W_OK) == 0));
    if (!has_drm_gles) {
        return "GPU unavailable: check Mesa EGL/GLES/GBM libs and /dev/dri/renderD* permissions.";
    }

    std::string cur_gov = read_text_token(DEVFREQ_GPU + "/governor");
    if (!cur_gov.empty() && cur_gov != "performance") {
        saved_gpu_gov = cur_gov;
        write_text_value(DEVFREQ_GPU + "/governor", "performance");
        gpu_gov_boosted = true;
    }
    pid_t pid = spawn_internal_worker("--internal-gpu-burn");
    if (pid > 0) {
        gpu_pid = pid;
        gpu_stress.store(true);
        return "Mali-G52 GPU Panfrost EGL/GLES stress STARTED (full-freq).";
    }
    return "GPU stress failed to fork.";
}

std::string RkTesterEngine::check_stress_health() {
    if (gpu_stress.load() && gpu_pid > 0) {
        int st = 0;
        if (waitpid(gpu_pid, &st, WNOHANG) == gpu_pid) {
            gpu_pid = 0;
            gpu_stress.store(false);
            if (gpu_gov_boosted) {
                write_text_value(DEVFREQ_GPU + "/governor", saved_gpu_gov);
                gpu_gov_boosted = false;
            }
            return "GPU stress exited unexpectedly (check Panfrost DRM / EGL / GLES).";
        }
    }
    if (npu_stress.load() && !npu_pids.empty()) {
        bool all_dead = true;
        for (pid_t& p : npu_pids) {
            if (p > 0) {
                int st = 0;
                if (waitpid(p, &st, WNOHANG) == p) {
                    p = 0;
                } else {
                    all_dead = false;
                }
            }
        }
        if (all_dead) {
            npu_pids.clear();
            npu_stress.store(false);
            if (npu_gov_boosted) {
                write_text_value(DEVFREQ_NPU + "/governor", saved_npu_gov);
                npu_gov_boosted = false;
            }
            return "NPU stress exited unexpectedly (check librknnrt.so / RKNPU driver).";
        }
    }
    if (rga_stress.load() && rga_pid > 0) {
        int st = 0;
        if (waitpid(rga_pid, &st, WNOHANG) == rga_pid) {
            rga_pid = 0;
            rga_stress.store(false);
            return "RGA stress exited unexpectedly (check librga.so / /dev/rga).";
        }
    }
    if (mpp_stress.load() && mpp_pid > 0) {
        int st = 0;
        if (waitpid(mpp_pid, &st, WNOHANG) == mpp_pid) {
            mpp_pid = 0;
            mpp_stress.store(false);
            return "RKMPP stress exited unexpectedly (check librockchip_mpp.so / /dev/mpp_service).";
        }
    }
    std::string ddr_warn = ddr_tester.check_health();
    if (!ddr_warn.empty()) return ddr_warn;
    return "";
}

std::string RkTesterEngine::toggle_npu_stress() {
    if (npu_stress.load()) {
        for (pid_t p : npu_pids) {
            if (p > 0) {
                stop_child_process(p);
            }
        }
        npu_pids.clear();
        npu_stress.store(false);
        if (npu_gov_boosted) {
            write_text_value(DEVFREQ_NPU + "/governor", saved_npu_gov);
            npu_gov_boosted = false;
        }
        return "NPU stress STOPPED.";
    }

    // 内置 librknnrt.so 双核 MatMul 原生压测，自动将 NPU 频率拉满至 950MHz
    std::string rknn_lib = find_shared_lib({"librknnrt.so"});
    if (rknn_lib.empty() || access(DEVFREQ_NPU.c_str(), F_OK) != 0) {
        return "NPU unavailable: check librknnrt.so and RKNPU driver.";
    }

    std::string cur_gov = read_text_token(DEVFREQ_NPU + "/governor");
    if (!cur_gov.empty() && cur_gov != "performance") {
        saved_npu_gov = cur_gov;
        write_text_value(DEVFREQ_NPU + "/governor", "performance");
        npu_gov_boosted = true;
    }
    npu_pids.clear();
    pid_t pid = spawn_internal_worker("--internal-npu-burn");
    if (pid > 0) {
        npu_pids.push_back(pid);
        npu_stress.store(true);
        return "RKNPU dual-core 6 TOPS MatMul stress STARTED (950MHz full-freq).";
    }
    return "NPU stress failed to fork.";
}

std::string RkTesterEngine::toggle_rga_stress() {
    if (rga_stress.load()) {
        stop_child_process(rga_pid);
        rga_stress.store(false);
        return "RGA 2D stress STOPPED.";
    }

    std::string rga_lib = find_shared_lib({"librga.so", "librga.so.2"});
    if (rga_lib.empty() || access("/dev/rga", R_OK | W_OK) != 0) {
        return "RGA unavailable: install librga.so and check /dev/rga permissions.";
    }

    pid_t pid = spawn_internal_worker("--internal-rga-burn");
    if (pid > 0) {
        rga_pid = pid;
        rga_stress.store(true);
        return "RGA2 dual-core 4K 2D stress STARTED (native built-in).";
    }
    return "RGA stress failed to fork.";
}

std::string RkTesterEngine::toggle_mpp_stress() {
    if (mpp_stress.load()) {
        stop_child_process(mpp_pid);
        mpp_stress.store(false);
        return "RKMPP video codec stress STOPPED.";
    }

    std::string mpp_lib = find_shared_lib({"librockchip_mpp.so.1", "librockchip_mpp.so", "librockchip_mpp.so.0"});
    if (mpp_lib.empty() || access("/dev/mpp_service", R_OK | W_OK) != 0) {
        return "RKMPP unavailable: check librockchip_mpp.so and /dev/mpp_service.";
    }

    pid_t pid = spawn_internal_worker("--internal-mpp-burn");
    if (pid > 0) {
        mpp_pid = pid;
        mpp_stress.store(true);
        return "RKMPP dual-4K H.265 VENC+VDEC stress STARTED (native built-in).";
    }
    return "RKMPP stress failed to fork.";
}

std::string RkTesterEngine::toggle_ddr_stress() {
    if (ddr_tester.active.load()) {
        ddr_tester.stop();
        return "DDR memory bandwidth & integrity stress STOPPED.";
    }
    if (cpu_burn_cores.empty()) rebuild_cpu_burn_list();
    ddr_tester.start(cpu_burn_cores);
    if (ddr_tester.active.load()) {
        return "DDR memory stress STARTED: " + ddr_tester.get_status_detail() + " NEON bit-flip verify.";
    }
    return "DDR stress unavailable: " + ddr_tester.get_status_detail();
}

std::string RkTesterEngine::toggle_all_stress() {
    auto stop_soc_parallel = [&]() {
        std::vector<pid_t> pids;
        if (gpu_pid > 0) pids.push_back(gpu_pid);
        for (pid_t p : npu_pids) { if (p > 0) pids.push_back(p); }
        if (rga_pid > 0) pids.push_back(rga_pid);
        if (mpp_pid > 0) pids.push_back(mpp_pid);
        pid_t ddr_p = ddr_tester.get_pid();
        if (ddr_p > 0) pids.push_back(ddr_p);

        for (pid_t p : pids) kill(-p, SIGTERM);
        cpu_stress.store(false);
        struct sched_param sp{};
        sp.sched_priority = 0;
        for (auto& t : cpu_threads) {
            if (t.joinable()) pthread_setschedparam(t.native_handle(), SCHED_OTHER, &sp);
        }

        // 并行回收全部子进程（总超时预算 250ms，杜绝逐个串行 4x400ms 卡顿）
        for (int i = 0; i < 50 && !pids.empty(); ++i) {
            for (auto it = pids.begin(); it != pids.end();) {
                if (waitpid(*it, nullptr, WNOHANG) == *it) it = pids.erase(it);
                else ++it;
            }
            if (!pids.empty()) usleep(5000);
        }
        for (pid_t p : pids) {
            kill(-p, SIGKILL);
            waitpid(p, nullptr, 0);
        }

        gpu_pid = 0; gpu_stress.store(false);
        if (gpu_gov_boosted) { write_text_value(DEVFREQ_GPU + "/governor", saved_gpu_gov); gpu_gov_boosted = false; }
        npu_pids.clear(); npu_stress.store(false);
        if (npu_gov_boosted) { write_text_value(DEVFREQ_NPU + "/governor", saved_npu_gov); npu_gov_boosted = false; }
        rga_pid = 0; rga_stress.store(false);
        mpp_pid = 0; mpp_stress.store(false);
        if (ddr_p > 0 || ddr_tester.active.load()) ddr_tester.notify_reaped();

        for (auto& t : cpu_threads) {
            if (t.joinable()) t.join();
        }
        cpu_threads.clear();
    };

    bool any_on = cpu_stress.load() || gpu_stress.load() || npu_stress.load() ||
                  rga_stress.load() || mpp_stress.load() || ddr_tester.active.load();
    bool all_on = cpu_stress.load() && gpu_stress.load() && npu_stress.load() &&
                  rga_stress.load() && mpp_stress.load() && ddr_tester.active.load();
    if (all_on) {
        stop_soc_parallel();
        return "All SoC+DDR stress (CPU/GPU/NPU/RGA/MPP/DDR) STOPPED.";
    }
    bool started_any = false;
    std::string warn;
    // 先 fork 独立子进程，再启动本进程 CPU 多线程，避免 fork 复制页表时争抢 mmap_lock
    if (!gpu_stress.load()) {
        (void)toggle_gpu_stress();
        if (gpu_stress.load()) started_any = true;
        else warn += " [GPU N/A]";
    }
    if (!npu_stress.load()) {
        (void)toggle_npu_stress();
        if (npu_stress.load()) started_any = true;
        else warn += " [NPU N/A]";
    }
    if (!rga_stress.load()) {
        (void)toggle_rga_stress();
        if (rga_stress.load()) started_any = true;
        else warn += " [RGA N/A]";
    }
    if (!mpp_stress.load()) {
        (void)toggle_mpp_stress();
        if (mpp_stress.load()) started_any = true;
        else warn += " [MPP N/A]";
    }
    if (!ddr_tester.active.load()) {
        (void)toggle_ddr_stress();
        if (ddr_tester.active.load()) started_any = true;
        else warn += " [DDR N/A]";
    }
    if (!cpu_stress.load()) {
        toggle_cpu_stress();
        started_any = started_any || cpu_stress.load();
    }
    if (!started_any && any_on) {
        // 可用项已全部运行，再次按 [4] 则统一并行停止
        stop_soc_parallel();
        return "All active SoC+DDR stress STOPPED.";
    }
    if (cpu_stress.load() && gpu_stress.load() && npu_stress.load() &&
        rga_stress.load() && mpp_stress.load() && ddr_tester.active.load()) {
        return "ALL STRESS ON: CPU + GPU + NPU + RGA + RKMPP + DDR at full load!";
    }
    return "SoC+DDR stress ON (available engines active" + warn + ").";
}

void RkTesterEngine::stop_all_stress() {
    // 阶段 1：向所有子进程和后台线程并行广播停止信号（零阻塞）
    net_emc.active.store(false);
    rs232.request_stop();
    rs485.request_stop();
    can_tester.request_stop();
    usb_tester.request_stop();
    cpu_stress.store(false);

    std::vector<pid_t> pids;
    if (gpu_pid > 0) pids.push_back(gpu_pid);
    for (pid_t p : npu_pids) { if (p > 0) pids.push_back(p); }
    if (rga_pid > 0) pids.push_back(rga_pid);
    if (mpp_pid > 0) pids.push_back(mpp_pid);
    pid_t ddr_p = ddr_tester.get_pid();
    if (ddr_p > 0) pids.push_back(ddr_p);

    for (pid_t p : pids) {
        kill(-p, SIGTERM);
    }

    // 同步将本进程 SCHED_IDLE CPU 烤机线程提权回 SCHED_OTHER，避免被抢占饿死
    struct sched_param sp{};
    sp.sched_priority = 0;
    for (auto& t : cpu_threads) {
        if (t.joinable()) pthread_setschedparam(t.native_handle(), SCHED_OTHER, &sp);
    }

    // 阶段 2：优先并行回收所有硬件压测子进程（总预算 250ms，释放全部高负载 SCHED_OTHER 进程）
    for (int i = 0; i < 50 && !pids.empty(); ++i) {
        for (auto it = pids.begin(); it != pids.end();) {
            if (waitpid(*it, nullptr, WNOHANG) == *it) it = pids.erase(it);
            else ++it;
        }
        if (!pids.empty()) usleep(5000);
    }
    for (pid_t p : pids) {
        kill(-p, SIGKILL);
        waitpid(p, nullptr, 0);
    }

    gpu_pid = 0;
    gpu_stress.store(false);
    if (gpu_gov_boosted) {
        write_text_value(DEVFREQ_GPU + "/governor", saved_gpu_gov);
        gpu_gov_boosted = false;
    }
    npu_pids.clear();
    npu_stress.store(false);
    if (npu_gov_boosted) {
        write_text_value(DEVFREQ_NPU + "/governor", saved_npu_gov);
        npu_gov_boosted = false;
    }
    rga_pid = 0;
    rga_stress.store(false);
    mpp_pid = 0;
    mpp_stress.store(false);
    if (ddr_p > 0 || ddr_tester.active.load()) {
        ddr_tester.notify_reaped();
    }

    // 阶段 3：统一回收本进程线程（外设排空窗口已在阶段 2 并行完成，此处几乎瞬时返回）
    for (auto& t : cpu_threads) {
        if (t.joinable()) t.join();
    }
    cpu_threads.clear();
    rs232.stop();
    rs485.stop();
    can_tester.stop();
    usb_tester.stop();
}

void RkTesterEngine::toggle_governor() {
    governor_perf = !governor_perf;
    const char* cpu_gov = governor_perf ? "performance" : "schedutil";
    for (int i = 0; i < num_cores; ++i) {
        std::string p = "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_governor";
        write_text_value(p, cpu_gov);
    }
    write_text_value(DEVFREQ_GPU + "/governor",
                     (governor_perf || gpu_stress.load()) ? "performance" : saved_gpu_gov);
    write_text_value(DEVFREQ_NPU + "/governor",
                     (governor_perf || npu_stress.load()) ? "performance" : saved_npu_gov);
    if (governor_perf) {
        gpu_gov_boosted = false;
        npu_gov_boosted = false;
    }
}

std::pair<double, std::vector<double>> RkTesterEngine::get_cpu_loads() {
    FILE* f = std::fopen("/proc/stat", "r");
    if (!f) return {smooth_total, smooth_cores};

    CpuTime cur_total;
    std::vector<CpuTime> cur_cores(num_cores);
    char line[256];
    while (std::fgets(line, sizeof(line), f)) {
        if (std::strncmp(line, "cpu", 3) != 0) break;
        char lbl[16]{};
        unsigned long long u = 0, n = 0, s = 0, id = 0, io = 0, ir = 0, so = 0;
        if (std::sscanf(line, "%15s %llu %llu %llu %llu %llu %llu %llu",
                        lbl, &u, &n, &s, &id, &io, &ir, &so) < 8) {
            continue;
        }
        unsigned long long tot = u + n + s + id + io + ir + so;
        unsigned long long idl = id + io;

        if (lbl[3] == '\0') {
            cur_total.total = tot;
            cur_total.idle = idl;
        } else if (lbl[3] >= '0' && lbl[3] <= '9') {
            int c_idx = std::atoi(lbl + 3);
            if (c_idx >= 0 && c_idx < num_cores) {
                cur_cores[c_idx].total = tot;
                cur_cores[c_idx].idle = idl;
            }
        }
    }
    std::fclose(f);

    double raw_total = 0.0;
    if (prev_total.total > 0 && cur_total.total > prev_total.total) {
        unsigned long long d_tot = cur_total.total - prev_total.total;
        unsigned long long d_idl = cur_total.idle - prev_total.idle;
        raw_total = (d_tot > d_idl) ? static_cast<double>(d_tot - d_idl) * 100.0 / d_tot : 0.0;
    }
    prev_total = cur_total;

    std::vector<double> raw_cores(num_cores, 0.0);
    for (int i = 0; i < num_cores; ++i) {
        if (prev_cores[i].total > 0 && cur_cores[i].total > prev_cores[i].total) {
            unsigned long long d_tot = cur_cores[i].total - prev_cores[i].total;
            unsigned long long d_idl = cur_cores[i].idle - prev_cores[i].idle;
            raw_cores[i] = (d_tot > d_idl) ? static_cast<double>(d_tot - d_idl) * 100.0 / d_tot : 0.0;
        }
        prev_cores[i] = cur_cores[i];
    }

    if (!cpu_initialized) {
        smooth_total = raw_total;
        smooth_cores = raw_cores;
        cpu_initialized = true;
    } else {
        smooth_total = 0.4 * raw_total + 0.6 * smooth_total;
        if (smooth_total < 0.5) smooth_total = 0.0;
        for (int i = 0; i < num_cores; ++i) {
            smooth_cores[i] = 0.4 * raw_cores[i] + 0.6 * smooth_cores[i];
            if (smooth_cores[i] < 0.5) smooth_cores[i] = 0.0;
        }
    }

    return {smooth_total, smooth_cores};
}

std::vector<int> RkTesterEngine::get_cpu_freqs() const {
    std::vector<int> freqs(num_cores, 0);
    for (int i = 0; i < num_cores; ++i) {
        char path[96];
        std::snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", i);
        long long khz = read_ll_file(path, 0);
        if (khz > 0) freqs[i] = static_cast<int>(khz / 1000);
    }
    return freqs;
}

std::pair<double, int> RkTesterEngine::get_gpu_stats() const {
    double load = 0.0;
    int load_freq_mhz = 0;
    FILE* f = std::fopen((DEVFREQ_GPU + "/load").c_str(), "r");
    if (f) {
        long long hz = 0;
        if (std::fscanf(f, "%lf@%lld", &load, &hz) >= 1 && hz > 0) {
            load_freq_mhz = static_cast<int>(hz / 1000000);
        }
        std::fclose(f);
    }
    // 优先读取真实硬件时钟 clk_gpu，其次读 devfreq/cur_freq，避免温控降频时 devfreq/load 内频率滞后失真
    int freq_mhz = read_debugfs_clk_mhz(DEBUG_CLK_GPU);
    if (freq_mhz <= 0) {
        long long hz = read_ll_file(DEVFREQ_GPU + "/cur_freq", 0);
        if (hz > 0) freq_mhz = static_cast<int>(hz / 1000000);
    }
    if (freq_mhz <= 0) freq_mhz = load_freq_mhz;
    return {load, freq_mhz};
}

std::pair<std::vector<double>, int> RkTesterEngine::get_npu_stats() const {
    std::vector<double> loads = {0.0, 0.0};
    FILE* f = std::fopen(DEBUG_RKNPU_LOAD.c_str(), "r");
    if (f) {
        char buf[256]{};
        size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
        std::fclose(f);
        buf[n] = '\0';
        const char* p = buf;
        while ((p = std::strstr(p, "Core")) != nullptr) {
            p += 4;
            char* end1 = nullptr;
            long core = std::strtol(p, &end1, 10);
            if (!end1 || end1 == p) continue;
            const char* colon = std::strchr(end1, ':');
            if (!colon) break;
            char* end2 = nullptr;
            double val = std::strtod(colon + 1, &end2);
            if (end2 && end2 != colon + 1 && core >= 0 && core < static_cast<long>(loads.size())) {
                loads[static_cast<size_t>(core)] = val;
            }
            p = colon + 1;
        }
    }
    long long hz = read_ll_file(DEVFREQ_NPU + "/cur_freq", 0);
    int freq_mhz = (hz > 0) ? static_cast<int>(hz / 1000000) : 0;
    return {loads, freq_mhz};
}

std::pair<std::vector<double>, int> RkTesterEngine::get_rga_stats() const {
    std::vector<double> loads = {0.0, 0.0};
    FILE* f = std::fopen(DEBUG_RKRGA_LOAD.c_str(), "r");
    if (f) {
        char line[160];
        int cur_idx = -1;
        while (std::fgets(line, sizeof(line), f)) {
            const char* sp = std::strstr(line, "scheduler[");
            if (sp) {
                cur_idx = std::atoi(sp + 10);
                continue;
            }
            const char* lp = std::strstr(line, "load");
            if (lp && cur_idx >= 0 && cur_idx < static_cast<int>(loads.size())) {
                const char* eq = std::strchr(lp, '=');
                if (eq) loads[static_cast<size_t>(cur_idx)] = std::strtod(eq + 1, nullptr);
            }
        }
        std::fclose(f);
    }
    int freq_mhz = read_debugfs_clk_mhz(DEBUG_CLK_RGA);
    return {loads, freq_mhz};
}

MppStats RkTesterEngine::get_mpp_stats() {
    MppStats st;
    uint64_t enc_irqs = 0, dec_irqs = 0;
    read_mpp_irq_counts(enc_irqs, dec_irqs);
    uint64_t now_us = get_time_us();

    if (mpp_irq_initialized && now_us > prev_mpp_sample_us) {
        double dt = static_cast<double>(now_us - prev_mpp_sample_us) / 1000000.0;
        if (dt >= 0.1) {
            double raw_enc = (enc_irqs >= prev_mpp_enc_irqs)
                                 ? static_cast<double>(enc_irqs - prev_mpp_enc_irqs) / dt
                                 : 0.0;
            double raw_dec = (dec_irqs >= prev_mpp_dec_irqs)
                                 ? static_cast<double>(dec_irqs - prev_mpp_dec_irqs) / dt
                                 : 0.0;
            smooth_mpp_enc_fps = (raw_enc < 0.1 && !mpp_stress.load()) ? 0.0
                                 : (0.5 * raw_enc + 0.5 * smooth_mpp_enc_fps);
            smooth_mpp_dec_fps = (raw_dec < 0.1 && !mpp_stress.load()) ? 0.0
                                 : (0.5 * raw_dec + 0.5 * smooth_mpp_dec_fps);
            if (smooth_mpp_enc_fps < 0.2) smooth_mpp_enc_fps = 0.0;
            if (smooth_mpp_dec_fps < 0.2) smooth_mpp_dec_fps = 0.0;
            prev_mpp_enc_irqs = enc_irqs;
            prev_mpp_dec_irqs = dec_irqs;
            prev_mpp_sample_us = now_us;
        }
    } else {
        prev_mpp_enc_irqs = enc_irqs;
        prev_mpp_dec_irqs = dec_irqs;
        prev_mpp_sample_us = now_us;
        mpp_irq_initialized = true;
    }

    st.enc_fps = smooth_mpp_enc_fps;
    st.dec_fps = smooth_mpp_dec_fps;
    st.enc_freq_mhz = read_debugfs_clk_mhz(DEBUG_CLK_VENC);
    st.dec_freq_mhz = read_debugfs_clk_mhz(DEBUG_CLK_VDEC);
    st.enc_total = (enc_irqs >= mpp_base_enc_irqs) ? (enc_irqs - mpp_base_enc_irqs) : 0;
    st.dec_total = (dec_irqs >= mpp_base_dec_irqs) ? (dec_irqs - mpp_base_dec_irqs) : 0;
    return st;
}

std::pair<uint64_t, uint64_t> RkTesterEngine::get_mpp_session_frames() const {
    uint64_t enc_irqs = 0, dec_irqs = 0;
    read_mpp_irq_counts(enc_irqs, dec_irqs);
    uint64_t enc_tot = (enc_irqs >= mpp_base_enc_irqs) ? (enc_irqs - mpp_base_enc_irqs) : 0;
    uint64_t dec_tot = (dec_irqs >= mpp_base_dec_irqs) ? (dec_irqs - mpp_base_dec_irqs) : 0;
    return {enc_tot, dec_tot};
}

std::map<std::string, double> RkTesterEngine::get_thermals() const {
    std::map<std::string, double> res;
    for (const auto& kv : thermal_zones) {
        long long raw_temp = read_ll_file(kv.second, -1);
        if (raw_temp >= 0) res[kv.first] = static_cast<double>(raw_temp) / 1000.0;
    }
    return res;
}

ThermalThrottleState RkTesterEngine::get_throttle_state() const {
    ThermalThrottleState st;
    for (const auto& kv : cooling_devs) {
        long long cur = read_ll_file(kv.second + "/cur_state", 0);
        long long maxv = read_ll_file(kv.second + "/max_state", 0);
        if (kv.first.find("cpu0") != std::string::npos) {
            st.cpu_little = (cur > 0);
        } else if (kv.first.find("cpu4") != std::string::npos) {
            st.cpu_big = (cur > 0);
        } else if (kv.first.find("gpu") != std::string::npos) {
            st.gpu = (cur > 0);
            st.gpu_cur = static_cast<int>(cur);
            st.gpu_max = static_cast<int>(maxv);
        } else if (kv.first.find("npu") != std::string::npos) {
            st.npu = (cur > 0);
        }
    }
    return st;
}

std::tuple<double, double, double> RkTesterEngine::get_mem_stats() const {
    FILE* f = std::fopen("/proc/meminfo", "r");
    if (!f) return {0.0, 0.0, 0.0};
    long long total = 0, avail = 0;
    char line[128];
    while (std::fgets(line, sizeof(line), f)) {
        if (std::strncmp(line, "MemTotal:", 9) == 0) {
            std::sscanf(line + 9, "%lld", &total);
        } else if (std::strncmp(line, "MemAvailable:", 13) == 0) {
            std::sscanf(line + 13, "%lld", &avail);
            if (total > 0) break;
        }
    }
    std::fclose(f);
    double tot_gb = total / (1024.0 * 1024.0);
    double used_gb = (total - avail) / (1024.0 * 1024.0);
    double pct = (total > 0) ? ((total - avail) * 100.0 / total) : 0.0;
    return {used_gb, tot_gb, pct};
}

std::string RkTesterEngine::get_uptime() const {
    FILE* f = std::fopen("/proc/uptime", "r");
    if (f) {
        double up = 0;
        int ok = std::fscanf(f, "%lf", &up);
        std::fclose(f);
        if (ok == 1) {
            long long s = static_cast<long long>(up);
            int hrs = static_cast<int>(s / 3600);
            int mins = static_cast<int>((s % 3600) / 60);
            int secs = static_cast<int>(s % 60);
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hrs, mins, secs);
            return buf;
        }
    }
    return "N/A";
}

void RkTesterEngine::init_governor_state() {
    std::string gov = read_text_token("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    governor_perf = (gov == "performance");
    std::string g_gov = read_text_token(DEVFREQ_GPU + "/governor");
    if (!g_gov.empty() && g_gov != "performance") saved_gpu_gov = g_gov;
    std::string n_gov = read_text_token(DEVFREQ_NPU + "/governor");
    if (!n_gov.empty() && n_gov != "performance") saved_npu_gov = n_gov;
}

void RkTesterEngine::init_thermals() {
    for (int i = 0; i < 16; ++i) {
        std::string t_path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/type";
        std::string val_path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/temp";
        std::string name = read_text_token(t_path);
        if (name.empty()) break;
        size_t pos = name.find("-thermal");
        if (pos != std::string::npos) name = name.substr(0, pos);
        thermal_zones[name] = val_path;
    }
}

void RkTesterEngine::init_cooling_devices() {
    cooling_devs.clear();
    for (int i = 0; i < 16; ++i) {
        std::string base = "/sys/class/thermal/cooling_device" + std::to_string(i);
        std::string type = read_text_token(base + "/type");
        if (type.empty()) break;
        cooling_devs[type] = base;
    }
}

void RkTesterEngine::rebuild_cpu_burn_list() {
    cpu_burn_cores.clear();
    bool isolated[64]{};
    if (keep_isolcpus) {
        std::string iso = read_text_token("/sys/devices/system/cpu/isolated");
        const char* p = iso.c_str();
        while (*p) {
            if (*p < '0' || *p > '9') {
                ++p;
                continue;
            }
            char* end1 = nullptr;
            long a = std::strtol(p, &end1, 10);
            long b = a;
            if (end1 && *end1 == '-') {
                char* end2 = nullptr;
                b = std::strtol(end1 + 1, &end2, 10);
                p = end2 ? end2 : (end1 + 1);
            } else {
                p = end1 ? end1 : (p + 1);
            }
            for (long c = a; c <= b && c < 64; ++c) {
                if (c >= 0) isolated[c] = true;
            }
        }
    }
    for (int c = 0; c < num_cores && c < 64; ++c) {
        if (!isolated[c]) cpu_burn_cores.push_back(c);
    }
    if (cpu_burn_cores.empty()) {
        for (int c = 0; c < num_cores; ++c) cpu_burn_cores.push_back(c);
    }
    target_cpu_cores = static_cast<int>(cpu_burn_cores.size());
}
