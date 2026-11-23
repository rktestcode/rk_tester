#pragma once

#include "config_store.hpp"
#include "soc_engine.hpp"

#include <map>
#include <string>
#include <vector>

struct UiState {
    double total_cpu{0.0};
    std::vector<double> core_cpus;
    std::vector<int> cpu_freqs;
    double gpu_load{0.0};
    int gpu_freq{0};
    std::vector<double> npu_loads{0.0, 0.0};
    int npu_freq{0};
    std::vector<double> rga_loads{0.0, 0.0};
    int rga_freq{0};
    double mpp_enc_fps{0.0};
    double mpp_dec_fps{0.0};
    int mpp_freq{0};
    double used_ram{0.0};
    double tot_ram{0.0};
    double ram_pct{0.0};
    std::map<std::string, double> thermals;
    std::string uptime;
    std::string config_path;

    bool menu_open{false};
    int menu_cursor{0};
    bool menu_cpu{false};
    bool menu_gpu{false};
    bool menu_npu{false};
    bool menu_rga{false};
    bool menu_mpp{false};
    bool menu_emc{false};
    bool menu_rs232{false};
    bool menu_rs485{false};
    bool menu_usb{false};
    bool menu_can{false};
    bool menu_ports[4]{true, true, true, true};

    std::string prompt_status;

    void sync_menu_from_engine(const RkTesterEngine& engine);
    void sync_menu_from_config(const SavedConfig& cfg);
    void apply_menu(RkTesterEngine& engine);
};

void print_usage();
void print_summary_report(const RkTesterEngine& engine);

void init_ncurses_ui();
void shutdown_ncurses_ui();
void run_tui_loop(RkTesterEngine& engine, UiState& ui);
