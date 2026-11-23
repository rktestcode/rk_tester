#include "soc_engine.hpp"
#include "common.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <pthread.h>
#include <regex>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

RkTesterEngine::RkTesterEngine() {
    num_cores = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    if (num_cores <= 0) num_cores = 8;
    target_cpu_cores = num_cores;
    prev_cores.resize(num_cores);
    smooth_cores.resize(num_cores, 0.0);

    init_governor_state();
    init_thermals();
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

std::string RkTesterEngine::toggle_gpu_stress() {
    if (gpu_stress.load()) {
        if (gpu_pid > 0) {
            kill(-gpu_pid, SIGTERM);
            bool reaped = false;
            for (int i = 0; i < 30; ++i) {
                if (waitpid(gpu_pid, nullptr, WNOHANG) == gpu_pid) {
                    reaped = true;
                    break;
                }
                usleep(10000);
            }
            if (!reaped) {
                kill(-gpu_pid, SIGKILL);
                waitpid(gpu_pid, nullptr, 0);
            }
            gpu_pid = 0;
        }
        gpu_stress.store(false);
        return "GPU stress STOPPED.";
    }

    // 1. 优先使用内置 Mali OpenCL 原生 GPU 计算压测（无需 X11/Wayland/glmark2，直接满频 900MHz 100%）
    std::string ocl_lib = find_shared_lib({"libMaliOpenCL.so.1", "libOpenCL.so.1"});
    if (!ocl_lib.empty() && access("/dev/mali0", R_OK | W_OK) == 0) {
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
            return "Mali-G52 GPU OpenCL stress STARTED (native built-in).";
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
            return "NPU stress exited unexpectedly (check librknnrt.so / RKNPU driver).";
        }
    }
    return "";
}

std::string RkTesterEngine::toggle_npu_stress() {
    if (npu_stress.load()) {
        for (pid_t p : npu_pids) {
            if (p > 0) {
                kill(-p, SIGKILL);
                waitpid(p, nullptr, 0);
            }
        }
        npu_pids.clear();
        npu_stress.store(false);
        return "NPU stress STOPPED.";
    }

    // 1. 优先使用内置 librknnrt.so 双核 MatMul 原生压测（无需 .rknn 模型与图片文件）
    std::string rknn_lib = find_shared_lib({"librknnrt.so"});
    if (!rknn_lib.empty() && access(DEVFREQ_NPU.c_str(), F_OK) == 0) {
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
            return "RKNPU dual-core 6 TOPS MatMul stress STARTED (native built-in).";
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

std::string RkTesterEngine::toggle_all_stress() {
    bool any_on = cpu_stress.load() || gpu_stress.load() || npu_stress.load();
    bool all_on = cpu_stress.load() && gpu_stress.load() && npu_stress.load();
    if (all_on) {
        if (cpu_stress.load()) toggle_cpu_stress();
        if (gpu_stress.load()) (void)toggle_gpu_stress();
        if (npu_stress.load()) (void)toggle_npu_stress();
        return "All compute stress STOPPED.";
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
    if (!started_any && any_on) {
        // 可用项已全部运行，再次按 [4] 则统一停止
        if (cpu_stress.load()) toggle_cpu_stress();
        if (gpu_stress.load()) (void)toggle_gpu_stress();
        if (npu_stress.load()) (void)toggle_npu_stress();
        return "All active compute stress STOPPED.";
    }
    if (cpu_stress.load() && gpu_stress.load() && npu_stress.load()) {
        return "ALL STRESS ON: CPU + GPU + NPU running at full load!";
    }
    return "Compute stress ON (CPU active" + warn + ").";
}

void RkTesterEngine::stop_all_stress() {
    net_emc.active.store(false);
    if (cpu_stress.load()) toggle_cpu_stress();
    if (gpu_stress.load()) (void)toggle_gpu_stress();
    if (npu_stress.load()) (void)toggle_npu_stress();
    rs232.stop();
    rs485.stop();
    can_tester.stop();
    usb_tester.stop();
}

void RkTesterEngine::toggle_governor() {
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
    std::ifstream f("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    std::string gov;
    if (f.is_open() && (f >> gov)) {
        governor_perf = (gov == "performance");
    }
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
