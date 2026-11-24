#include "config_store.hpp"
#include "soc_engine.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <unistd.h>

std::string get_config_path() {
    if (const char* ev = std::getenv("RK_TESTER_CONF"); ev && ev[0] != '\0') {
        return ev;
    }
    if (geteuid() == 0) {
        return "/etc/rk_tester.conf";
    }
    if (const char* home = std::getenv("HOME"); home && home[0] != '\0') {
        return std::string(home) + "/.rk_tester.conf";
    }
    return "/tmp/rk_tester.conf";
}

SavedConfig load_config(const std::string& path) {
    SavedConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq);
        std::string v = line.substr(eq + 1);
        while (!v.empty() && (v.back() == '\r' || v.back() == ' ')) v.pop_back();
        int iv = 0;
        try { iv = std::stoi(v); } catch (...) { continue; }
        if (k == "cpu") cfg.cpu = (iv != 0);
        else if (k == "gpu") cfg.gpu = (iv != 0);
        else if (k == "npu") cfg.npu = (iv != 0);
        else if (k == "rga") cfg.rga = (iv != 0);
        else if (k == "mpp") cfg.mpp = (iv != 0);
        else if (k == "emc") cfg.emc = (iv != 0);
        else if (k == "rs232") cfg.rs232 = (iv != 0);
        else if (k == "rs485") cfg.rs485 = (iv != 0);
        else if (k == "usb") cfg.usb = (iv != 0);
        else if (k == "can") cfg.can = (iv != 0);
        else if (k == "lan0") cfg.ports[0] = (iv != 0);
        else if (k == "lan1") cfg.ports[1] = (iv != 0);
        else if (k == "lan2") cfg.ports[2] = (iv != 0);
        else if (k == "lan3") cfg.ports[3] = (iv != 0);
        else if (k == "rs232_dir" && iv >= 0 && iv <= 2) cfg.rs232_dir = static_cast<SerialDirection>(iv);
        else if (k == "rs485_dir" && iv >= 0 && iv <= 2) cfg.rs485_dir = static_cast<SerialDirection>(iv);
        else if (k == "can_dir" && iv >= 0 && iv <= 2) cfg.can_dir = static_cast<CanDirection>(iv);
        else if (k == "governor_perf") cfg.governor_perf = (iv != 0);
        cfg.loaded = true;
    }
    return cfg;
}

bool save_config_snapshot(const std::string& path, const SavedConfig& cfg) {
    std::string tmp_path = path + ".tmp";
    std::ofstream f(tmp_path, std::ios::out | std::ios::trunc);
    if (!f.is_open()) return false;
    f << "# RK3576 rk_tester saved test selection\n"
      << "cpu=" << (cfg.cpu ? 1 : 0) << "\n"
      << "gpu=" << (cfg.gpu ? 1 : 0) << "\n"
      << "npu=" << (cfg.npu ? 1 : 0) << "\n"
      << "rga=" << (cfg.rga ? 1 : 0) << "\n"
      << "mpp=" << (cfg.mpp ? 1 : 0) << "\n"
      << "emc=" << (cfg.emc ? 1 : 0) << "\n"
      << "rs232=" << (cfg.rs232 ? 1 : 0) << "\n"
      << "rs485=" << (cfg.rs485 ? 1 : 0) << "\n"
      << "usb=" << (cfg.usb ? 1 : 0) << "\n"
      << "can=" << (cfg.can ? 1 : 0) << "\n"
      << "lan0=" << (cfg.ports[0] ? 1 : 0) << "\n"
      << "lan1=" << (cfg.ports[1] ? 1 : 0) << "\n"
      << "lan2=" << (cfg.ports[2] ? 1 : 0) << "\n"
      << "lan3=" << (cfg.ports[3] ? 1 : 0) << "\n"
      << "rs232_dir=" << static_cast<int>(cfg.rs232_dir) << "\n"
      << "rs485_dir=" << static_cast<int>(cfg.rs485_dir) << "\n"
      << "can_dir=" << static_cast<int>(cfg.can_dir) << "\n"
      << "governor_perf=" << (cfg.governor_perf ? 1 : 0) << "\n";
    f.flush();
    bool ok = f.good();
    f.close();
    if (!ok) {
        unlink(tmp_path.c_str());
        return false;
    }
    return rename(tmp_path.c_str(), path.c_str()) == 0;
}

void save_engine_config(const RkTesterEngine& engine, const std::string& path) {
    SavedConfig cfg;
    cfg.cpu = engine.cpu_stress.load();
    cfg.gpu = engine.gpu_stress.load();
    cfg.npu = engine.npu_stress.load();
    cfg.rga = engine.rga_stress.load();
    cfg.mpp = engine.mpp_stress.load();
    cfg.emc = engine.net_emc.active.load();
    cfg.rs232 = engine.rs232.active.load();
    cfg.rs485 = engine.rs485.active.load();
    cfg.usb = engine.usb_tester.active.load();
    cfg.can = engine.can_tester.active.load();
    for (int i = 0; i < 4; ++i) cfg.ports[i] = engine.get_port_enabled(static_cast<size_t>(i));
    cfg.rs232_dir = engine.rs232.dir.load();
    cfg.rs485_dir = engine.rs485.dir.load();
    cfg.can_dir = engine.can_tester.dir.load();
    cfg.governor_perf = engine.governor_perf;
    save_config_snapshot(path, cfg);
}
