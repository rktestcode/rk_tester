#include "soc_engine.hpp"
#include "common.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <pthread.h>
#include <regex>
#include <sched.h>
#include <sstream>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>
#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace {

std::string read_text_token(const std::string& path) {
    std::ifstream f(path);
    std::string s;
    if (f.is_open() && (f >> s)) return s;
    return "";
}

void write_text_value(const std::string& path, const std::string& val) {
    std::ofstream f(path);
    if (f.is_open()) f << val;
}

int read_debugfs_clk_mhz(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return 0;
    long long hz = 0;
    if (f >> hz && hz > 0) {
        return static_cast<int>(hz / 1000000);
    }
    return 0;
}

void read_mpp_irq_counts(uint64_t& enc_irqs, uint64_t& dec_irqs) {
    enc_irqs = 0;
    dec_irqs = 0;
    std::ifstream f("/proc/interrupts");
    if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
        bool is_enc = (line.find("rkvenc") != std::string::npos);
        bool is_dec = (line.find("rkvdec") != std::string::npos);
        if (!is_enc && !is_dec) continue;
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::istringstream ss(line.substr(colon + 1));
        std::string tok;
        uint64_t sum = 0;
        while (ss >> tok) {
            bool all_digits = !tok.empty();
            for (char c : tok) {
                if (c < '0' || c > '9') {
                    all_digits = false;
                    break;
                }
            }
            if (!all_digits) break;
            try {
                sum += std::stoull(tok);
            } catch (...) {
                break;
            }
        }
        if (is_enc) enc_irqs += sum;
        if (is_dec) dec_irqs += sum;
    }
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

    init_governor_state();
    init_thermals();
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

void RkTesterEngine::toggle_cpu_stress() {
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
                    // Phase 1: 高位翻转率 12-way NEON FMA 满发射算力烤机
                    for (int k = 0; k < 24000; ++k) {
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

                    // Phase 2: NEON 向量流式读写 (L1/L2/DDR 带宽压测)
                    constexpr size_t kChunk = 65536; // 256KB per pass, sweeps full 4MB
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
                    for (int k = 0; k < 24000; ++k) {
                        for (int u = 0; u < 8; ++u) {
                            acc[u] = std::fmaf(acc[u], 0.751953125f, 0.3712345f);
                            acc[u] = 1.8734567f - acc[u] * 0.751953125f;
                        }
                    }
                    constexpr size_t kChunk = 32768;
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

    // 1. 优先使用内置 Mali OpenCL 原生 GPU 计算压测（无需 X11/Wayland/glmark2，直接满频 900MHz 100%）
    std::string ocl_lib = find_shared_lib({"libMaliOpenCL.so.1", "libOpenCL.so.1"});
    if (!ocl_lib.empty() && access("/dev/mali0", R_OK | W_OK) == 0) {
        std::string cur_gov = read_text_token(DEVFREQ_GPU + "/governor");
        if (!cur_gov.empty() && cur_gov != "performance") {
            saved_gpu_gov = cur_gov;
            write_text_value(DEVFREQ_GPU + "/governor", "performance");
            gpu_gov_boosted = true;
        }
        pid_t pid = fork();
        if (pid == 0) {
            setsid();
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDOUT_FILENO);
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            execl("/proc/self/exe", "rk_tester", "--internal-gpu-burn", nullptr);
            _exit(127);
        } else if (pid > 0) {
            gpu_pid = pid;
            gpu_stress.store(true);
            return "Mali-G52 GPU OpenCL stress STARTED (900MHz full-freq).";
        }
        return "GPU stress failed to fork.";
    }

    // 2. 回退外部 glmark2 / kmscube
    std::string gpu_bin;
    bool use_glmark_args = true;
    if (const char* env_bin = std::getenv("RK_GPU_BIN"); env_bin && env_bin[0] != '\0' && binary_in_path(env_bin)) {
        gpu_bin = env_bin;
        use_glmark_args = (gpu_bin.find("glmark2") != std::string::npos);
    } else {
        const char* cands[] = {"glmark2-es2-drm", "glmark2-es2-wayland", "glmark2-es2", "kmscube"};
        for (const char* c : cands) {
            if (binary_in_path(c)) {
                gpu_bin = c;
                use_glmark_args = (std::string(c).find("glmark2") != std::string::npos);
                break;
            }
        }
    }

    if (gpu_bin.empty()) {
        return "GPU unavailable: install libmali (OpenCL) or glmark2-es2-drm.";
    }

    pid_t pid = fork();
    if (pid == 0) {
        setsid();
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        if (use_glmark_args) {
            execlp(gpu_bin.c_str(), gpu_bin.c_str(), "--off-screen", "--run-forever", nullptr);
        } else {
            execlp(gpu_bin.c_str(), gpu_bin.c_str(), nullptr);
        }
        _exit(127);
    } else if (pid > 0) {
        gpu_pid = pid;
        gpu_stress.store(true);
        return "Mali-G52 GPU stress STARTED (" + gpu_bin + ").";
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
            return "GPU stress exited unexpectedly (check libmali / OpenCL / DRM).";
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

    // 1. 优先使用内置 librknnrt.so 双核 MatMul 原生压测，并强制将 NPU 频率拉满至 950MHz
    std::string rknn_lib = find_shared_lib({"librknnrt.so"});
    if (!rknn_lib.empty() && access(DEVFREQ_NPU.c_str(), F_OK) == 0) {
        std::string cur_gov = read_text_token(DEVFREQ_NPU + "/governor");
        if (!cur_gov.empty() && cur_gov != "performance") {
            saved_npu_gov = cur_gov;
            write_text_value(DEVFREQ_NPU + "/governor", "performance");
            npu_gov_boosted = true;
        }
        npu_pids.clear();
        pid_t pid = fork();
        if (pid == 0) {
            setsid();
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDOUT_FILENO);
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            execl("/proc/self/exe", "rk_tester", "--internal-npu-burn", nullptr);
            _exit(127);
        } else if (pid > 0) {
            npu_pids.push_back(pid);
            npu_stress.store(true);
            return "RKNPU dual-core 6 TOPS MatMul stress STARTED (950MHz full-freq).";
        }
        return "NPU stress failed to fork.";
    }

    // 2. 回退外部 rknn_common_test / rknn_benchmark
    auto pick_file = [](const char* env_key, const std::vector<std::string>& cands, int mode) -> std::string {
        if (const char* ev = std::getenv(env_key); ev && ev[0] != '\0' && access(ev, mode) == 0) {
            return ev;
        }
        for (const auto& p : cands) {
            if (access(p.c_str(), mode) == 0) return p;
        }
        return "";
    };

    std::string bin = pick_file("RK_NPU_BIN", {
        RKNN_TEST_BIN, "/usr/local/bin/rknn_common_test",
        "/usr/bin/rknn_benchmark", "/usr/local/bin/rknn_benchmark"
    }, X_OK);
    std::string model = pick_file("RK_NPU_MODEL", {
        RKNN_MODEL, "/usr/share/model/mobilenet_v1.rknn",
        "/usr/share/rknn_model_zoo/mobilenet_v1.rknn"
    }, R_OK);
    std::string img = pick_file("RK_NPU_IMG", {
        RKNN_IMG, "/usr/share/rknn_model_zoo/dog_224x224.jpg"
    }, R_OK);

    bool is_benchmark = (bin.find("rknn_benchmark") != std::string::npos);
    if (bin.empty() || model.empty() || (!is_benchmark && img.empty())) {
        return "NPU unavailable: install librknnrt.so or rknn_common_test + .rknn model.";
    }

    npu_pids.clear();
    for (int i = 0; i < 2; ++i) {
        pid_t pid = fork();
        if (pid == 0) {
            setsid();
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDOUT_FILENO);
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            while (true) {
                auto t0 = std::chrono::steady_clock::now();
                pid_t child = fork();
                if (child == 0) {
                    if (is_benchmark) {
                        execl(bin.c_str(), bin.c_str(), model.c_str(), "500", nullptr);
                    } else {
                        execl(bin.c_str(), bin.c_str(), model.c_str(), img.c_str(), "500", nullptr);
                    }
                    _exit(127);
                } else if (child > 0) {
                    int st = 0;
                    waitpid(child, &st, 0);
                    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now() - t0)
                                          .count();
                    if (!WIFEXITED(st) || WEXITSTATUS(st) != 0 || elapsed_ms < 100) {
                        usleep(500000);
                    }
                } else {
                    usleep(500000);
                }
            }
        } else if (pid > 0) {
            npu_pids.push_back(pid);
        }
    }
    if (!npu_pids.empty()) {
        npu_stress.store(true);
        return "RKNPU dual-core 6 TOPS stress STARTED.";
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

    pid_t pid = fork();
    if (pid == 0) {
        setsid();
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        execl("/proc/self/exe", "rk_tester", "--internal-rga-burn", nullptr);
        _exit(127);
    } else if (pid > 0) {
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
    if (!mpp_lib.empty() && access("/dev/mpp_service", R_OK | W_OK) == 0) {
        pid_t pid = fork();
        if (pid == 0) {
            setsid();
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDOUT_FILENO);
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            execl("/proc/self/exe", "rk_tester", "--internal-mpp-burn", nullptr);
            _exit(127);
        } else if (pid > 0) {
            mpp_pid = pid;
            mpp_stress.store(true);
            return "RKMPP dual-4K H.265 VENC+VDEC stress STARTED (native built-in).";
        }
        return "RKMPP stress failed to fork.";
    }

    // 回退外部 mpi_enc_test
    if (binary_in_path("mpi_enc_test") && access("/dev/mpp_service", R_OK | W_OK) == 0) {
        pid_t pid = fork();
        if (pid == 0) {
            setsid();
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDOUT_FILENO);
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            execlp("mpi_enc_test", "mpi_enc_test", "-w", "3840", "-h", "2160", "-t", "16777220", "-n", "0", nullptr);
            _exit(127);
        } else if (pid > 0) {
            mpp_pid = pid;
            mpp_stress.store(true);
            return "RKMPP 4K H.265 stress STARTED (mpi_enc_test).";
        }
        return "RKMPP stress failed to fork.";
    }

    return "RKMPP unavailable: install librockchip_mpp.so and check /dev/mpp_service.";
}

std::string RkTesterEngine::toggle_all_stress() {
    bool any_on = cpu_stress.load() || gpu_stress.load() || npu_stress.load() ||
                  rga_stress.load() || mpp_stress.load();
    bool all_on = cpu_stress.load() && gpu_stress.load() && npu_stress.load() &&
                  rga_stress.load() && mpp_stress.load();
    if (all_on) {
        if (cpu_stress.load()) toggle_cpu_stress();
        if (gpu_stress.load()) (void)toggle_gpu_stress();
        if (npu_stress.load()) (void)toggle_npu_stress();
        if (rga_stress.load()) (void)toggle_rga_stress();
        if (mpp_stress.load()) (void)toggle_mpp_stress();
        return "All SoC stress (CPU/GPU/NPU/RGA/MPP) STOPPED.";
    }
    bool started_any = false;
    std::string warn;
    if (!cpu_stress.load()) {
        toggle_cpu_stress();
        started_any = started_any || cpu_stress.load();
    }
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
    if (!started_any && any_on) {
        // 可用项已全部运行，再次按 [4] 则统一停止
        if (cpu_stress.load()) toggle_cpu_stress();
        if (gpu_stress.load()) (void)toggle_gpu_stress();
        if (npu_stress.load()) (void)toggle_npu_stress();
        if (rga_stress.load()) (void)toggle_rga_stress();
        if (mpp_stress.load()) (void)toggle_mpp_stress();
        return "All active SoC stress STOPPED.";
    }
    if (cpu_stress.load() && gpu_stress.load() && npu_stress.load() &&
        rga_stress.load() && mpp_stress.load()) {
        return "ALL STRESS ON: CPU + GPU + NPU + RGA + RKMPP at full load!";
    }
    return "SoC stress ON (available engines active" + warn + ").";
}

void RkTesterEngine::stop_all_stress() {
    net_emc.active.store(false);
    if (cpu_stress.load()) toggle_cpu_stress();
    if (gpu_stress.load()) (void)toggle_gpu_stress();
    if (npu_stress.load()) (void)toggle_npu_stress();
    if (rga_stress.load()) (void)toggle_rga_stress();
    if (mpp_stress.load()) (void)toggle_mpp_stress();
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
    std::ifstream stat_file("/proc/stat");
    if (!stat_file.is_open()) return {smooth_total, smooth_cores};

    std::string line;
    CpuTime cur_total;
    std::vector<CpuTime> cur_cores(num_cores);

    while (std::getline(stat_file, line)) {
        if (line.rfind("cpu", 0) != 0) break;
        std::istringstream ss(line);
        std::string cpu_lbl;
        unsigned long long u = 0, n = 0, s = 0, id = 0, io = 0, ir = 0, so = 0;
        if (!(ss >> cpu_lbl >> u >> n >> s >> id >> io >> ir >> so)) continue;
        unsigned long long tot = u + n + s + id + io + ir + so;
        unsigned long long idl = id + io;

        if (cpu_lbl == "cpu") {
            cur_total.total = tot;
            cur_total.idle = idl;
        } else if (cpu_lbl.size() > 3) {
            try {
                int c_idx = std::stoi(cpu_lbl.substr(3));
                if (c_idx >= 0 && c_idx < num_cores) {
                    cur_cores[c_idx].total = tot;
                    cur_cores[c_idx].idle = idl;
                }
            } catch (...) {}
        }
    }

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
        std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq";
        std::ifstream f(path);
        if (f.is_open()) {
            int khz = 0;
            if (f >> khz) freqs[i] = khz / 1000;
        }
    }
    return freqs;
}

std::pair<double, int> RkTesterEngine::get_gpu_stats() const {
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
                    freq_mhz = static_cast<int>(std::stoll(f_str) / 1000000);
                } catch (...) {}
            }
        }
    }
    if (freq_mhz == 0) {
        std::ifstream cf(DEVFREQ_GPU + "/cur_freq");
        if (cf.is_open()) {
            long long hz = 0;
            if (cf >> hz) freq_mhz = static_cast<int>(hz / 1000000);
        }
    }
    return {load, freq_mhz};
}

std::pair<std::vector<double>, int> RkTesterEngine::get_npu_stats() const {
    static const std::regex re("Core(\\d+):\\s*(\\d+)%");
    std::vector<double> loads = {0.0, 0.0};
    int freq_mhz = 0;
    std::ifstream f(DEBUG_RKNPU_LOAD);
    if (f.is_open()) {
        std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        auto begin = std::sregex_iterator(content.begin(), content.end(), re);
        auto end = std::sregex_iterator();
        for (auto it = begin; it != end; ++it) {
            int core = std::stoi((*it)[1].str());
            double val = std::stod((*it)[2].str());
            if (core >= 0 && core < static_cast<int>(loads.size())) {
                loads[core] = val;
            }
        }
    }
    std::ifstream cf(DEVFREQ_NPU + "/cur_freq");
    if (cf.is_open()) {
        long long hz = 0;
        if (cf >> hz) freq_mhz = static_cast<int>(hz / 1000000);
    }
    return {loads, freq_mhz};
}

std::pair<std::vector<double>, int> RkTesterEngine::get_rga_stats() const {
    std::vector<double> loads = {0.0, 0.0};
    std::ifstream f(DEBUG_RKRGA_LOAD);
    if (f.is_open()) {
        std::string line;
        int cur_idx = -1;
        while (std::getline(f, line)) {
            size_t sp = line.find("scheduler[");
            if (sp != std::string::npos) {
                try {
                    cur_idx = std::stoi(line.substr(sp + 10));
                } catch (...) {
                    cur_idx = -1;
                }
                continue;
            }
            size_t lp = line.find("load");
            if (lp != std::string::npos && cur_idx >= 0 && cur_idx < static_cast<int>(loads.size())) {
                size_t eq = line.find('=', lp);
                if (eq != std::string::npos) {
                    try {
                        loads[cur_idx] = std::stod(line.substr(eq + 1));
                    } catch (...) {}
                }
            }
        }
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
        std::ifstream f(kv.second);
        if (f.is_open()) {
            int raw_temp = 0;
            if (f >> raw_temp) res[kv.first] = raw_temp / 1000.0;
        }
    }
    return res;
}

std::tuple<double, double, double> RkTesterEngine::get_mem_stats() const {
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

std::string RkTesterEngine::get_uptime() const {
    std::ifstream f("/proc/uptime");
    double up = 0;
    if (f >> up) {
        long long s = static_cast<long long>(up);
        int hrs = static_cast<int>(s / 3600);
        int mins = static_cast<int>((s % 3600) / 60);
        int secs = static_cast<int>(s % 60);
        char buf[32];
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hrs, mins, secs);
        return buf;
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
