#include "accel_burn.hpp"
#include "can_tester.hpp"
#include "common.hpp"
#include "config_store.hpp"
#include "serial_tester.hpp"
#include "soc_engine.hpp"
#include "tui.hpp"

#include <clocale>
#include <csignal>
#include <string>
#include <unistd.h>

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");

    // 先解析命令行参数（确保 --help 零副作用且无需 root 权限）
    bool start_in_menu = false;
    bool no_restore = false;
    bool cli_rs232_dir = false, cli_rs485_dir = false, cli_can_dir = false;
    SerialDirection init_rs232_dir = SerialDirection::A_TO_B;
    SerialDirection init_rs485_dir = SerialDirection::A_TO_B;
    CanDirection init_can_dir = CanDirection::CAN0_TO_CAN1;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--internal-gpu-burn") {
            signal(SIGTERM, sig_handler);
            signal(SIGINT, sig_handler);
            return run_internal_gpu_opencl_burn();
        } else if (arg == "--internal-npu-burn") {
            signal(SIGTERM, sig_handler);
            signal(SIGINT, sig_handler);
            return run_internal_npu_matmul_burn();
        } else if (arg == "--internal-rga-burn") {
            signal(SIGTERM, sig_handler);
            signal(SIGINT, sig_handler);
            return run_internal_rga_burn();
        } else if (arg == "--internal-mpp-burn") {
            signal(SIGTERM, sig_handler);
            signal(SIGINT, sig_handler);
            return run_internal_mpp_burn();
        } else if (arg == "-h" || arg == "--help") {
            print_usage();
            return 0;
        } else if (arg == "-m" || arg == "--menu") {
            start_in_menu = true;
        } else if (arg == "--no-restore") {
            no_restore = true;
        } else if (arg == "--rs232-0to1") {
            init_rs232_dir = SerialDirection::A_TO_B; cli_rs232_dir = true;
        } else if (arg == "--rs232-1to0") {
            init_rs232_dir = SerialDirection::B_TO_A; cli_rs232_dir = true;
        } else if (arg == "--rs232-pingpong") {
            init_rs232_dir = SerialDirection::PING_PONG; cli_rs232_dir = true;
        } else if (arg == "--rs485-2to3") {
            init_rs485_dir = SerialDirection::A_TO_B; cli_rs485_dir = true;
        } else if (arg == "--rs485-3to2") {
            init_rs485_dir = SerialDirection::B_TO_A; cli_rs485_dir = true;
        } else if (arg == "--rs485-pingpong") {
            init_rs485_dir = SerialDirection::PING_PONG; cli_rs485_dir = true;
        } else if (arg == "--can-0to1") {
            init_can_dir = CanDirection::CAN0_TO_CAN1; cli_can_dir = true;
        } else if (arg == "--can-1to0") {
            init_can_dir = CanDirection::CAN1_TO_CAN0; cli_can_dir = true;
        } else if (arg == "--can-pingpong") {
            init_can_dir = CanDirection::PING_PONG; cli_can_dir = true;
        }
    }

    // 注册异步信号安全的退出处理器
    struct sigaction sa{};
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    std::string cfg_path = get_config_path();
    SavedConfig saved_cfg = no_restore ? SavedConfig{} : load_config(cfg_path);

    RkTesterEngine engine;
    engine.rs232.set_direction(cli_rs232_dir ? init_rs232_dir
                                             : (saved_cfg.loaded ? saved_cfg.rs232_dir : init_rs232_dir));
    engine.rs485.set_direction(cli_rs485_dir ? init_rs485_dir
                                             : (saved_cfg.loaded ? saved_cfg.rs485_dir : init_rs485_dir));
    engine.can_tester.set_direction(cli_can_dir ? init_can_dir
                                                : (saved_cfg.loaded ? saved_cfg.can_dir : init_can_dir));

    if (saved_cfg.loaded) {
        for (size_t i = 0; i < 4; ++i) {
            engine.set_port_enabled(i, saved_cfg.ports[i]);
        }
        if (saved_cfg.governor_perf != engine.governor_perf) {
            engine.toggle_governor();
        }
    }

    init_ncurses_ui();

    engine.get_cpu_loads();
    usleep(150000);

    UiState ui;
    ui.config_path = cfg_path;
    ui.core_cpus.assign(engine.num_cores, 0.0);
    ui.cpu_freqs.assign(engine.num_cores, 0);
    ui.menu_open = start_in_menu;

    if (start_in_menu) {
        if (saved_cfg.loaded) {
            ui.sync_menu_from_config(saved_cfg);
            ui.prompt_status = "Loaded last selection (" + cfg_path + "). Press [S] to run.";
        } else {
            ui.sync_menu_from_engine(engine);
            ui.prompt_status = "Test Items Menu opened. Use [Space] to toggle, [S] to apply.";
        }
    } else if (saved_cfg.loaded) {
        // 自动恢复并启动上次选中的测试项
        ui.sync_menu_from_config(saved_cfg);
        ui.apply_menu(engine);
        int active_cnt = (engine.cpu_stress.load() ? 1 : 0) + (engine.gpu_stress.load() ? 1 : 0) +
                         (engine.npu_stress.load() ? 1 : 0) + (engine.rga_stress.load() ? 1 : 0) +
                         (engine.mpp_stress.load() ? 1 : 0) + (engine.net_emc.active.load() ? 1 : 0) +
                         (engine.rs232.active.load() ? 1 : 0) + (engine.rs485.active.load() ? 1 : 0) +
                         (engine.usb_tester.active.load() ? 1 : 0) + (engine.can_tester.active.load() ? 1 : 0);
        if (ui.prompt_status.find("unavailable") == std::string::npos) {
            ui.prompt_status = "Restored last selection: " + std::to_string(active_cnt) + " active (" + cfg_path + ").";
        }
    } else {
        ui.prompt_status = "Ready. Press [M] Menu, [1-4,0,V] SoC, [5] LAN, [6] 232, [7] 485, [8] USB, [9] CAN.";
    }

    run_tui_loop(engine, ui);

    engine.stop_all_stress();
    shutdown_ncurses_ui();
    print_summary_report(engine);

    return 0;
}
