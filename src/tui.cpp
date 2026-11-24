#include "tui.hpp"
#include "common.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <ncurses.h>
#include <tuple>
#include <unistd.h>

void UiState::sync_menu_from_engine(const RkTesterEngine& engine) {
    menu_cpu = engine.cpu_stress.load();
    menu_gpu = engine.gpu_stress.load();
    menu_npu = engine.npu_stress.load();
    menu_rga = engine.rga_stress.load();
    menu_mpp = engine.mpp_stress.load();
    menu_ddr = engine.ddr_tester.active.load();
    menu_emc = engine.net_emc.active.load();
    menu_rs232 = engine.rs232.active.load();
    menu_rs485 = engine.rs485.active.load();
    menu_usb = engine.usb_tester.active.load();
    menu_can = engine.can_tester.active.load();
    for (int i = 0; i < 4; ++i) {
        menu_ports[i] = engine.get_port_enabled(static_cast<size_t>(i));
    }
}

void UiState::sync_menu_from_config(const SavedConfig& cfg) {
    menu_cpu = cfg.cpu;
    menu_gpu = cfg.gpu;
    menu_npu = cfg.npu;
    menu_rga = cfg.rga;
    menu_mpp = cfg.mpp;
    menu_ddr = cfg.ddr;
    menu_emc = cfg.emc;
    menu_rs232 = cfg.rs232;
    menu_rs485 = cfg.rs485;
    menu_usb = cfg.usb;
    menu_can = cfg.can;
    for (int i = 0; i < 4; ++i) menu_ports[i] = cfg.ports[i];
}

void UiState::apply_menu(RkTesterEngine& engine) {
    std::string warn;
    // 启动顺序：先 fork 独立硬件压测子进程（GPU/NPU/RGA/MPP/DDR），再启外设线程，最后启本进程 CPU 8 核线程，
    // 彻底避免 fork/pthread_create 争抢 mmap_lock 导致界面卡顿
    if (menu_gpu != engine.gpu_stress.load()) {
        std::string msg = engine.toggle_gpu_stress();
        if (menu_gpu && !engine.gpu_stress.load()) warn += " | " + msg;
    }
    if (menu_npu != engine.npu_stress.load()) {
        std::string msg = engine.toggle_npu_stress();
        if (menu_npu && !engine.npu_stress.load()) warn += " | " + msg;
    }
    if (menu_rga != engine.rga_stress.load()) {
        std::string msg = engine.toggle_rga_stress();
        if (menu_rga && !engine.rga_stress.load()) warn += " | " + msg;
    }
    if (menu_mpp != engine.mpp_stress.load()) {
        std::string msg = engine.toggle_mpp_stress();
        if (menu_mpp && !engine.mpp_stress.load()) warn += " | " + msg;
    }
    if (menu_ddr != engine.ddr_tester.active.load()) {
        std::string msg = engine.toggle_ddr_stress();
        if (menu_ddr && !engine.ddr_tester.active.load()) warn += " | " + msg;
    }
    if (menu_emc != engine.net_emc.active.load()) {
        engine.net_emc.active.store(menu_emc);
        if (menu_emc) {
            std::string m = engine.net_emc.high_load.load()
                                ? "4-LAN High-Load Burst (1480B) STARTED from Menu"
                                : "4-LAN Dual-Pair Mutual Ping STARTED from Menu";
            engine.log_emc_event("INFO", m, 1);
        } else {
            engine.log_emc_event("INFO", "4-LAN Ping test PAUSED from Menu", 2);
        }
    }
    if (menu_rs232 != engine.rs232.active.load()) engine.toggle_rs232();
    if (menu_rs485 != engine.rs485.active.load()) engine.toggle_rs485();
    if (menu_usb != engine.usb_tester.active.load()) engine.toggle_usb_stress();
    if (menu_can != engine.can_tester.active.load()) engine.toggle_can();
    if (menu_cpu != engine.cpu_stress.load()) engine.toggle_cpu_stress();
    for (size_t i = 0; i < 4; ++i) {
        engine.set_port_enabled(i, menu_ports[i]);
    }

    if (!config_path.empty()) {
        save_engine_config(engine, config_path);
    }

    int active_cnt = (engine.cpu_stress.load() ? 1 : 0) + (engine.gpu_stress.load() ? 1 : 0) +
                     (engine.npu_stress.load() ? 1 : 0) + (engine.rga_stress.load() ? 1 : 0) +
                     (engine.mpp_stress.load() ? 1 : 0) + (engine.ddr_tester.active.load() ? 1 : 0) +
                     (engine.net_emc.active.load() ? 1 : 0) +
                     (engine.rs232.active.load() ? 1 : 0) + (engine.rs485.active.load() ? 1 : 0) +
                     (engine.usb_tester.active.load() ? 1 : 0) + (engine.can_tester.active.load() ? 1 : 0);
    prompt_status = "Saved & applied: " + std::to_string(active_cnt) + " active" + warn;
    menu_open = false;
}

void print_usage() {
    std::cout << "====================================================================\n"
              << "       RK3576 Unified Hardware Test Platform (rk_tester)\n"
              << "====================================================================\n\n"
              << "[Usage]\n"
              << "  sudo rk_tester                    (Launch & auto-restore last selection)\n"
              << "  sudo rk_tester -m                 (Open Menu pre-filled with last selection)\n"
              << "  sudo rk_tester --no-restore       (Launch clean standby without restoring)\n"
              << "  sudo rk_tester --keep-isolcpus    (Leave RT isolated CPUs e.g. 6-7 unburned)\n"
              << "  sudo rk_tester --lan-highload     (Start 4-LAN in 1480B Gigabit Burst mode)\n"
              << "  sudo rk_tester --can-fd           (Start CAN in 64B 2Mbps CAN-FD stress mode)\n"
              << "  sudo rk_tester --rs232-0to1|1to0|pingpong  (Set RS232 direction)\n"
              << "  sudo rk_tester --rs485-2to3|3to2|pingpong  (Set RS485 direction)\n"
              << "  sudo rk_tester --can-0to1|1to0|pingpong    (Set CAN direction)\n\n"
              << "[Default 4-LAN Configurations (Dual-Pair Mutual Ping / Burst)]\n"
              << "  Pair 1: Port 1 (lan0: 192.168.10.1/24) <-> Port 2 (lan1: 192.168.11.1/24)\n"
              << "  Pair 2: Port 3 (lan2: 192.168.20.1/24) <-> Port 4 (lan3: 192.168.21.1/24)\n"
              << "  Engine: Layer-2 AF_PACKET raw frames (bypasses kernel loopback)\n\n"
              << "[Keyboard Controls]\n"
              << "  [M] Open Test Items Selection Menu (Interactive Checklist)\n"
              << "  [1] Toggle CPU 8-Core Compute Stress\n"
              << "  [2] Toggle Mali-G52 GPU Panfrost EGL/GLES Stress\n"
              << "  [3] Toggle RKNPU Dual-Core 6 TOPS Neural MatMul Stress\n"
              << "  [0] Toggle RGA2 Dual-Core 4K 2D CSC/Rotate Stress\n"
              << "  [V] Toggle RKMPP Dual-4K H.265 VENC+VDEC Codec Stress\n"
              << "  [B] Toggle Large-Capacity DDR Bandwidth & Bit-Flip Stress\n"
              << "  [4] [ALL STRESS] Full Load on CPU + GPU + NPU + RGA + RKMPP + DDR\n"
              << "  [5] Toggle 4-LAN Dual-Pair Mutual Ping / Gigabit Burst\n"
              << "  [T] Toggle 4-LAN Mode (100ms EMC Monitor <-> 1480B Gigabit Burst)\n"
              << "  [6] Toggle RS232 Serial Test (ttyS0 / ttyS1)\n"
              << "  [7] Toggle RS485 Serial Test (ttyS2 / ttyS3)\n"
              << "  [8] Toggle 4-Port USB Storage 4MB O_DIRECT R/W Stress (sda..sdd)\n"
              << "  [9] Toggle CAN Bus Test (can0 / can1)\n"
              << "  [F] Toggle CAN Mode (CAN 2.0 8B 50fps <-> CAN-FD 64B 2Mbps 500fps)\n"
              << "  [Z] Switch RS232 Direction (S0->S1 / S1->S0 / Ping-Pong)\n"
              << "  [X] Switch RS485 Direction (S2->S3 / S3->S2 / Ping-Pong)\n"
              << "  [D] Switch CAN Direction   (c0->c1 / c1->c0 / Ping-Pong)\n"
              << "  [R] Reset Statistics (All items)\n"
              << "  [P] Switch CPU/GPU/NPU Governor (Performance / Schedutil)\n"
              << "  [I] Re-initialize 4-LAN Static IPs & Raw Sockets\n"
              << "  [Q] Quit & Print Final Test Report\n"
              << "====================================================================\n";
}

void print_summary_report(const RkTesterEngine& engine) {
    EmcSnapshot emc = engine.get_emc_snapshot();

    std::cout << "\n====================================================================\n";
    std::cout << "      RK3576 Hardware Test Summary (4-LAN + Serial + CAN + USB)\n";
    std::cout << "====================================================================\n";
    std::cout << " Test Mode          : Dual-Pair "
              << (emc.high_load ? "High-Load Gigabit Burst (1480B, lan0<->lan1 & lan2<->lan3)"
                                : "Mutual Ping (100ms EMC, lan0<->lan1 & lan2<->lan3)")
              << "\n";

    double tot_loss_pct = (emc.total_sent > 0) ? (emc.total_lost * 100.0 / emc.total_sent) : 0.0;

    if (emc.total_sent == 0) {
        std::cout << " Overall Immunity   : NOT TESTED (4-LAN ping test was not started, 0 packets sent)\n";
    } else if (emc.total_flaps == 0 && emc.total_lost == 0) {
        std::cout << " Overall Immunity   : CLASS A (PASS: 0 Packet Loss, 0 Link Flaps)\n";
    } else if (emc.total_flaps == 0 && tot_loss_pct < 1.0) {
        std::cout << " Overall Immunity   : CLASS B (MARGINAL: Transient loss < 1%, Self-recovered)\n";
    } else {
        std::cout << " Overall Immunity   : CLASS C (FAIL: Link Flap or Excessive Packet Loss)\n";
    }

    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " 4-LAN Port Ping & Loss Breakdown:\n";
    for (const auto& p : emc.ports) {
        double p_loss = (p.sent > 0) ? (p.lost * 100.0 / p.sent) : 0.0;
        double avg_rtt = (p.recv > 0) ? (p.rtt_sum / p.recv) : 0.0;
        char buf[192];
        snprintf(buf, sizeof(buf),
                 " * %-19s (IP: %-13s): Sent:%-5llu Recv:%-5llu Lost:%-3llu (%4.1f%%) Flaps:%-2llu AvgRTT:%4.2fms",
                 p.ch_name.c_str(), p.current_ip.c_str(),
                 static_cast<unsigned long long>(p.sent),
                 static_cast<unsigned long long>(p.recv),
                 static_cast<unsigned long long>(p.lost),
                 p_loss, static_cast<unsigned long long>(p.flaps), avg_rtt);
        std::cout << buf;
        if (p.hw_rx_err > 0 || p.hw_tx_err > 0 || p.hw_rx_drop > 0) {
            std::cout << " [HW err r=" << p.hw_rx_err << "/t=" << p.hw_tx_err << "/d=" << p.hw_rx_drop << "]";
        }
        if (!p.enabled) {
            std::cout << " [DISABLED]\n";
        } else if (!p.carrier && p.sent == 0) {
            std::cout << " [NOT CONNECTED]\n";
        } else if (p.sent == 0) {
            std::cout << " [NOT TESTED]\n";
        } else if (p.flaps == 0 && p.lost == 0 && p.recv > 0) {
            std::cout << " [PASS]\n";
        } else {
            std::cout << " [FAIL/GLITCH]\n";
        }
    }

    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " Serial Port Transmission Summary (115200 8N1, 64B Frame):\n";
    auto print_serial_summary = [](const SerialChannel& sc) {
        double tx_kb = sc.tx_bytes.load() / 1024.0;
        double rx_kb = sc.rx_bytes.load() / 1024.0;
        uint64_t tx_f = sc.tx_frames.load();
        uint64_t rx_f = sc.rx_frames.load();
        uint64_t err_f = sc.err_frames.load();
        uint64_t crc_f = sc.crc_errors.load();
        uint64_t oe_c = sc.hw_oe.load();
        uint64_t fe_c = sc.hw_fe.load();
        uint64_t pe_c = sc.hw_pe.load();
        double loss_pct = (tx_f > 0) ? (err_f * 100.0 / tx_f) : 0.0;
        double avg_rtt = (rx_f > 0) ? (sc.rtt_sum_ms.load() / rx_f) : 0.0;
        double max_rtt = sc.rtt_max_ms.load();

        std::cout << " * " << sc.get_mode_name() << ":\n";
        if (sc.state.load() == ChannelState::PORT_ERROR) {
            std::cout << "   - Status   : [PORT ERROR] Failed to open " << sc.get_error_detail() << "\n";
            return;
        }
        if (tx_f == 0) {
            std::cout << "   - Status   : [NOT TESTED]\n";
            return;
        }
        char buf[320];
        snprintf(buf, sizeof(buf),
                 "   - Traffic  : TX: %.1f KB (%llu pkts) | RX: %.1f KB (%llu pkts)\n"
                 "   - Integrity: Loss/Err: %llu (%4.1f%%) | CRC Err: %llu | HW(oe/fe/pe): %llu/%llu/%llu | RTT Avg/Max: %.1f/%.1f ms",
                 tx_kb, static_cast<unsigned long long>(tx_f),
                 rx_kb, static_cast<unsigned long long>(rx_f),
                 static_cast<unsigned long long>(err_f), loss_pct,
                 static_cast<unsigned long long>(crc_f),
                 static_cast<unsigned long long>(oe_c),
                 static_cast<unsigned long long>(fe_c),
                 static_cast<unsigned long long>(pe_c),
                 avg_rtt, max_rtt);
        std::cout << buf;
        if (err_f == 0 && crc_f == 0 && rx_f > 0) {
            std::cout << " [PASS]\n";
        } else {
            std::cout << " [FAIL/ERRORS]\n";
        }
    };
    print_serial_summary(engine.rs232);
    print_serial_summary(engine.rs485);

    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " CAN Bus (can0 / can1) Transmission Summary (" << engine.can_tester.get_bitrate_info()
              << ", " << (engine.can_tester.fd_mode.load() ? "64B CAN-FD Frame" : "8B Frame") << "):\n";
    std::string mode_str = engine.can_tester.get_mode_name();
    uint64_t can_tx = engine.can_tester.tx_frames.load();
    uint64_t can_rx = engine.can_tester.rx_frames.load();
    double can_tx_kb = engine.can_tester.tx_bytes.load() / 1024.0;
    double can_rx_kb = engine.can_tester.rx_bytes.load() / 1024.0;
    uint64_t can_err = engine.can_tester.err_frames.load();
    uint64_t can_crc = engine.can_tester.crc_errors.load();
    uint64_t can_bus = engine.can_tester.bus_errors.load();
    double can_loss_pct = (can_tx > 0) ? (can_err * 100.0 / can_tx) : 0.0;
    double can_avg_rtt = (can_rx > 0) ? (engine.can_tester.rtt_sum_ms.load() / can_rx) : 0.0;
    double can_max_rtt = engine.can_tester.rtt_max_ms.load();
    std::cout << " * Mode Configured  : " << mode_str << "\n";
    if (engine.can_tester.state.load() == ChannelState::PORT_ERROR) {
        std::cout << " * Total CAN Result : [CAN OPEN ERROR] Failed to open " << engine.can_tester.get_error_detail() << "\n";
    } else if (can_tx == 0) {
        std::cout << " * Total CAN Result : [NOT TESTED]\n";
    } else {
        char cbuf[256];
        snprintf(cbuf, sizeof(cbuf),
                 " * Traffic          : TX: %.1f KB (%llu pkts) | RX Verified: %.1f KB (%llu pkts)\n"
                 " * Integrity        : Drops: %llu (%4.1f%%) | CRC Err: %llu | Bus Err: %llu | RTT Avg/Max: %.2f/%.2f ms\n",
                 can_tx_kb, static_cast<unsigned long long>(can_tx),
                 can_rx_kb, static_cast<unsigned long long>(can_rx),
                 static_cast<unsigned long long>(can_err), can_loss_pct,
                 static_cast<unsigned long long>(can_crc),
                 static_cast<unsigned long long>(can_bus),
                 can_avg_rtt, can_max_rtt);
        std::cout << cbuf;
        std::cout << " * Total CAN Result : ";
        if (can_err == 0 && can_crc == 0 && can_bus == 0 && can_rx > 0) {
            std::cout << "[ALL PASS - 0 ERRORS]\n";
        } else {
            std::cout << "[FAIL - " << can_err << " DROPS, " << can_crc << " CRC, " << can_bus << " BUS ERR]\n";
        }
    }

    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " 4-Port USB Storage Read/Write Stress Detailed Summary:\n";
    for (size_t i = 0; i < engine.usb_tester.disks.size(); ++i) {
        const auto& d = engine.usb_tester.disks[i];
        auto meta = d->get_meta();
        uint64_t w_mb = d->write_bytes.load() / (1024 * 1024);
        uint64_t r_mb = d->read_bytes.load() / (1024 * 1024);
        uint64_t w_ops = d->write_ops.load();
        uint64_t r_ops = d->read_ops.load();
        uint64_t errs = d->errors.load();

        std::string mod = meta.model.empty() ? "USB Storage Device" : meta.model;
        char cap_buf[32];
        if (meta.capacity_gb > 0) snprintf(cap_buf, sizeof(cap_buf), " (%.1f GB)", meta.capacity_gb);
        else cap_buf[0] = '\0';

        std::cout << " * Port " << (i + 1) << " (" << meta.dev_name << "): " << mod << cap_buf << "\n";
        std::cout << "   - Mount Point : " << d->mount_point
                  << " (" << (meta.fs_type.empty() ? "auto" : meta.fs_type)
                  << ", Part: " << (meta.part_name.empty() ? "N/A" : meta.part_name) << ")\n";
        if (w_mb == 0 && r_mb == 0 && errs == 0) {
            if (!d->mounted.load()) {
                std::cout << "   - Status      : [NOT CONNECTED / UNPLUGGED]\n";
            } else {
                std::cout << "   - Status      : [NOT TESTED] (Test was not started on this device)\n";
            }
        } else {
            std::cout << "   - Data Tested : Written: " << w_mb << " MB (" << w_ops
                      << " blocks) | Read: " << r_mb << " MB (" << r_ops << " blocks)\n";
            std::cout << "   - Verification: Errors: " << errs;
            if (d->disconnects.load() > 0) {
                std::cout << " (Unplug/IO failures: " << d->disconnects.load()
                          << ", Auto-recovered: " << d->recovers.load() << ")";
            }
            if (errs == 0) std::cout << " [PASS]\n";
            else std::cout << " [FAIL/ERRORS]\n";
        }
    }

    uint64_t tot_usb_w = engine.usb_tester.get_total_write_mb();
    uint64_t tot_usb_r = engine.usb_tester.get_total_read_mb();
    uint64_t tot_usb_err = engine.usb_tester.get_total_errors();
    std::cout << " Total USB Storage  : Written: " << tot_usb_w << " MB | Read: " << tot_usb_r
              << " MB | Errors: " << tot_usb_err;
    if (tot_usb_w > 0 && tot_usb_err == 0) std::cout << " [ALL PASS]\n";
    else if (tot_usb_w > 0 || tot_usb_err > 0) std::cout << " [FAIL]\n";
    else std::cout << " [NOT TESTED]\n";

    std::cout << "--------------------------------------------------------------------\n";
    uint64_t ddr_bytes = engine.ddr_tester.get_bytes_tested();
    uint64_t ddr_alloc = engine.ddr_tester.get_alloc_mb();
    uint64_t ddr_passes = engine.ddr_tester.get_passes();
    uint64_t ddr_errs = engine.ddr_tester.get_bit_errors();
    if (ddr_bytes > 0 || ddr_alloc > 0) {
        double ddr_gb = static_cast<double>(ddr_bytes) / (1024.0 * 1024.0 * 1024.0);
        char dbuf[192];
        snprintf(dbuf, sizeof(dbuf),
                 " DDR Memory Stress  : Region: %llu MB | Tested: %.2f GB (%llu passes) | Bit Errors: %llu %s\n",
                 static_cast<unsigned long long>(ddr_alloc), ddr_gb,
                 static_cast<unsigned long long>(ddr_passes),
                 static_cast<unsigned long long>(ddr_errs),
                 (ddr_errs == 0 && ddr_bytes > 0) ? "[PASS]" : "[FAIL]");
        std::cout << dbuf;
    }
    auto [mpp_enc_tot, mpp_dec_tot] = engine.get_mpp_session_frames();
    if (mpp_enc_tot > 0 || mpp_dec_tot > 0) {
        std::cout << " RKMPP Video Codec  : Hardware Encoded: " << mpp_enc_tot
                  << " frames | Hardware Decoded: " << mpp_dec_tot << " frames\n";
    }
    std::cout << " Total LAN Ping Sent: " << emc.total_sent << " packets | Received: " << emc.total_recv
              << " | Lost: " << emc.total_lost << " (" << std::fixed << std::setprecision(1) << tot_loss_pct << " %)";
    if (emc.total_tx_bytes > 0 || emc.total_rx_bytes > 0) {
        double tx_mb = static_cast<double>(emc.total_tx_bytes) / (1024.0 * 1024.0);
        double rx_mb = static_cast<double>(emc.total_rx_bytes) / (1024.0 * 1024.0);
        std::cout << " | TX: " << std::fixed << std::setprecision(1) << tx_mb
                  << " MB / RX: " << rx_mb << " MB";
    }
    std::cout << "\n";
    std::cout << " Detailed Log Path  : " << engine.net_emc.log_path << "\n";
    std::cout << "====================================================================\n\n";
}

namespace {

double get_thermal_val(const std::map<std::string, double>& thermals, const std::string& key) {
    auto it = thermals.find(key);
    return (it != thermals.end()) ? it->second : 0.0;
}

void draw_bar(int y, int x, int width, double percent, int color_pair) {
    double clamped = std::max(0.0, std::min(100.0, percent));
    int bar_chars = std::max(4, width - 8);
    int filled = static_cast<int>(bar_chars * clamped / 100.0);
    int empty = bar_chars - filled;

    attron(COLOR_PAIR(7));
    mvaddch(y, x, '[');
    attroff(COLOR_PAIR(7));

    attron(color_pair | A_BOLD);
    for (int i = 0; i < filled; ++i) mvaddch(y, x + 1 + i, '#');
    attroff(color_pair | A_BOLD);

    attron(COLOR_PAIR(8));
    for (int i = 0; i < empty; ++i) mvaddch(y, x + 1 + filled + i, '.');
    attroff(COLOR_PAIR(8));

    attron(COLOR_PAIR(7));
    mvprintw(y, x + 1 + bar_chars, "]%4.0f%%", percent);
    attroff(COLOR_PAIR(7));
}

void draw_section_banner(int y, int w, int badge_pair, int line_pair, const char* badge, const char* subinfo = "") {
    int x = 2;
    // 无色块背景：使用终端默认背景 + 高对比度粗体彩色标题，避免黑/灰字贴亮底看不清
    attron(COLOR_PAIR(line_pair));
    mvprintw(y, x, "== ");
    attroff(COLOR_PAIR(line_pair));
    x += 3;

    attron(COLOR_PAIR(badge_pair) | A_BOLD);
    mvprintw(y, x, "%s", badge);
    attroff(COLOR_PAIR(badge_pair) | A_BOLD);
    x += static_cast<int>(std::strlen(badge));

    attron(COLOR_PAIR(line_pair));
    mvprintw(y, x, " ==");
    attroff(COLOR_PAIR(line_pair));
    x += 3;

    if (subinfo && std::strlen(subinfo) > 0 && x + 4 < w - 2) {
        mvprintw(y, x, " ");
        x += 1;
        int max_sub = std::max(0, w - 3 - x);
        attron(COLOR_PAIR(7));
        mvprintw(y, x, "%.*s", max_sub, subinfo);
        attroff(COLOR_PAIR(7));
        x += std::min(static_cast<int>(std::strlen(subinfo)), max_sub);
        if (x < w - 2) {
            mvprintw(y, x, " ");
            x += 1;
        }
    }

    if (x < w - 2) {
        attron(COLOR_PAIR(line_pair));
        mvhline(y, x, '-', w - 2 - x);
        attroff(COLOR_PAIR(line_pair));
    }
}

int get_color_by_load(double pct) {
    if (pct > 80.0) return COLOR_PAIR(3);
    if (pct > 40.0) return COLOR_PAIR(2);
    return COLOR_PAIR(1);
}

// 返回 true 表示请求退出程序
bool handle_menu_input(int ch, RkTesterEngine& engine, UiState& ui) {
    constexpr int kMenuItems = 17; // 0..10 test items, 11..14 LAN filter, 15 Apply, 16 Cancel
    if (ch == 'q' || ch == 'Q') {
        return true;
    } else if (ch == 27 || ch == 'm' || ch == 'M') {
        ui.menu_open = false;
        ui.prompt_status = "Menu closed. No changes applied.";
    } else if (ch == KEY_UP || ch == 'k' || ch == 'K') {
        ui.menu_cursor = (ui.menu_cursor + kMenuItems - 1) % kMenuItems;
    } else if (ch == KEY_DOWN || ch == 'j' || ch == 'J') {
        ui.menu_cursor = (ui.menu_cursor + 1) % kMenuItems;
    } else if (ch == KEY_LEFT) {
        if (ui.menu_cursor >= 12 && ui.menu_cursor <= 14) ui.menu_cursor--;
        else if (ui.menu_cursor == 16) ui.menu_cursor = 15;
    } else if (ch == KEY_RIGHT) {
        if (ui.menu_cursor >= 11 && ui.menu_cursor <= 13) ui.menu_cursor++;
        else if (ui.menu_cursor == 15) ui.menu_cursor = 16;
    } else if (ch == ' ' || ch == '\n' || ch == '\r' || ch == KEY_ENTER) {
        if (ui.menu_cursor == 0) ui.menu_cpu = !ui.menu_cpu;
        else if (ui.menu_cursor == 1) ui.menu_gpu = !ui.menu_gpu;
        else if (ui.menu_cursor == 2) ui.menu_npu = !ui.menu_npu;
        else if (ui.menu_cursor == 3) ui.menu_rga = !ui.menu_rga;
        else if (ui.menu_cursor == 4) ui.menu_mpp = !ui.menu_mpp;
        else if (ui.menu_cursor == 5) ui.menu_ddr = !ui.menu_ddr;
        else if (ui.menu_cursor == 6) ui.menu_emc = !ui.menu_emc;
        else if (ui.menu_cursor == 7) ui.menu_rs232 = !ui.menu_rs232;
        else if (ui.menu_cursor == 8) ui.menu_rs485 = !ui.menu_rs485;
        else if (ui.menu_cursor == 9) ui.menu_usb = !ui.menu_usb;
        else if (ui.menu_cursor == 10) ui.menu_can = !ui.menu_can;
        else if (ui.menu_cursor >= 11 && ui.menu_cursor <= 14) ui.menu_ports[ui.menu_cursor - 11] = !ui.menu_ports[ui.menu_cursor - 11];
        else if (ui.menu_cursor == 15) ui.apply_menu(engine);
        else if (ui.menu_cursor == 16) {
            ui.menu_open = false;
            ui.prompt_status = "Menu closed. No changes applied.";
        }
    } else if (ch == '1') {
        ui.menu_cpu = !ui.menu_cpu; ui.menu_cursor = 0;
    } else if (ch == '2') {
        ui.menu_gpu = !ui.menu_gpu; ui.menu_cursor = 1;
    } else if (ch == '3') {
        ui.menu_npu = !ui.menu_npu; ui.menu_cursor = 2;
    } else if (ch == '0') {
        ui.menu_rga = !ui.menu_rga; ui.menu_cursor = 3;
    } else if (ch == 'v' || ch == 'V') {
        ui.menu_mpp = !ui.menu_mpp; ui.menu_cursor = 4;
    } else if (ch == 'b' || ch == 'B') {
        ui.menu_ddr = !ui.menu_ddr; ui.menu_cursor = 5;
    } else if (ch == '4') {
        bool all_on = ui.menu_cpu && ui.menu_gpu && ui.menu_npu && ui.menu_rga && ui.menu_mpp && ui.menu_ddr;
        ui.menu_cpu = ui.menu_gpu = ui.menu_npu = ui.menu_rga = ui.menu_mpp = ui.menu_ddr = !all_on;
    } else if (ch == '5') {
        ui.menu_emc = !ui.menu_emc; ui.menu_cursor = 6;
    } else if (ch == '6') {
        ui.menu_rs232 = !ui.menu_rs232; ui.menu_cursor = 7;
    } else if (ch == '7') {
        ui.menu_rs485 = !ui.menu_rs485; ui.menu_cursor = 8;
    } else if (ch == '8') {
        ui.menu_usb = !ui.menu_usb; ui.menu_cursor = 9;
    } else if (ch == '9') {
        ui.menu_can = !ui.menu_can; ui.menu_cursor = 10;
    } else if (ch == 't' || ch == 'T') {
        engine.net_emc.toggle_high_load();
        ui.prompt_status = engine.net_emc.high_load.load()
                               ? "4-LAN mode: High-Load 1480B Gigabit Burst (paired ports only)."
                               : "4-LAN mode: Standard 100ms EMC Monitor.";
    } else if (ch == 'f' || ch == 'F') {
        engine.can_tester.toggle_fd_mode();
        ui.prompt_status = engine.can_tester.fd_mode.load()
                               ? "CAN mode: CAN-FD 64B 2Mbps high-rate (500fps)."
                               : "CAN mode: Standard CAN 2.0 8B 500kbps (50fps).";
    } else if (ch == 'z' || ch == 'Z') {
        engine.rs232.cycle_direction();
        ui.prompt_status = "RS232 mode switched: " + engine.rs232.get_mode_name();
    } else if (ch == 'x' || ch == 'X') {
        engine.rs485.cycle_direction();
        ui.prompt_status = "RS485 mode switched: " + engine.rs485.get_mode_name();
    } else if (ch == 'd' || ch == 'D') {
        if (ui.menu_cursor == 7) {
            engine.rs232.cycle_direction();
            ui.prompt_status = "RS232 mode switched: " + engine.rs232.get_mode_name();
        } else if (ui.menu_cursor == 8) {
            engine.rs485.cycle_direction();
            ui.prompt_status = "RS485 mode switched: " + engine.rs485.get_mode_name();
        } else {
            engine.can_tester.cycle_direction();
            ui.prompt_status = "CAN mode switched: " + engine.can_tester.get_mode_name();
        }
    } else if (ch == 'a' || ch == 'A') {
        ui.menu_cpu = ui.menu_gpu = ui.menu_npu = ui.menu_rga = ui.menu_mpp = ui.menu_ddr = true;
        ui.menu_emc = ui.menu_rs232 = ui.menu_rs485 = ui.menu_usb = ui.menu_can = true;
        ui.menu_ports[0] = ui.menu_ports[1] = ui.menu_ports[2] = ui.menu_ports[3] = true;
    } else if (ch == 'n' || ch == 'N') {
        ui.menu_cpu = ui.menu_gpu = ui.menu_npu = ui.menu_rga = ui.menu_mpp = ui.menu_ddr = false;
        ui.menu_emc = ui.menu_rs232 = ui.menu_rs485 = ui.menu_usb = ui.menu_can = false;
    } else if (ch == 's' || ch == 'S') {
        ui.apply_menu(engine);
    }
    return false;
}

bool handle_dashboard_input(int ch, RkTesterEngine& engine, UiState& ui) {
    if (ch == 'q' || ch == 'Q' || ch == 27) {
        return true;
    } else if (ch == 'm' || ch == 'M') {
        ui.menu_open = true;
        ui.menu_cursor = 0;
        ui.sync_menu_from_engine(engine);
        ui.prompt_status = "Test Items Menu opened. Use [Up/Down], [Space], [S] to apply.";
    } else if (ch == '1' || ch == 'c' || ch == 'C') {
        engine.toggle_cpu_stress();
        ui.prompt_status = engine.cpu_stress.load() ? "CPU 8-core compute stress STARTED." : "CPU stress STOPPED.";
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '2' || ch == 'g' || ch == 'G') {
        ui.prompt_status = engine.toggle_gpu_stress();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '3' || ch == 'n' || ch == 'N') {
        ui.prompt_status = engine.toggle_npu_stress();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '0') {
        ui.prompt_status = engine.toggle_rga_stress();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 'v' || ch == 'V') {
        ui.prompt_status = engine.toggle_mpp_stress();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 'b' || ch == 'B') {
        ui.prompt_status = engine.toggle_ddr_stress();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '4' || ch == 'a' || ch == 'A') {
        ui.prompt_status = engine.toggle_all_stress();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '5' || ch == 'e' || ch == 'E') {
        bool next = !engine.net_emc.active.load();
        engine.net_emc.active.store(next);
        if (next) {
            if (engine.net_emc.high_load.load()) {
                ui.prompt_status = "4-LAN High-Load 1480B Gigabit Burst STARTED (paired ports only).";
                engine.log_emc_event("INFO", "4-LAN High-Load 1480B Gigabit Burst STARTED", 1);
            } else {
                ui.prompt_status = "4-LAN Dual-Pair Mutual Ping STARTED (lan0<->lan1, lan2<->lan3).";
                engine.log_emc_event("INFO", "4-LAN Dual-Pair Mutual Ping STARTED (lan0<->lan1, lan2<->lan3)", 1);
            }
        } else {
            ui.prompt_status = "4-LAN ping monitoring PAUSED.";
            engine.log_emc_event("INFO", "4-LAN ping monitoring PAUSED.", 2);
        }
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 't' || ch == 'T') {
        engine.net_emc.toggle_high_load();
        bool hl = engine.net_emc.high_load.load();
        ui.prompt_status = hl
                               ? "4-LAN switched to HIGH-LOAD 1480B Gigabit Burst (paired ports only)."
                               : "4-LAN switched to standard 100ms EMC Monitor.";
        engine.log_emc_event("INFO", hl ? "Switched 4-LAN to HIGH-LOAD 1480B Burst"
                                        : "Switched 4-LAN to 100ms EMC Monitor", 1);
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '6') {
        engine.toggle_rs232();
        ui.prompt_status = engine.rs232.active.load()
                               ? (engine.rs232.get_mode_name() + " STARTED.")
                               : "RS232 test STOPPED.";
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '7') {
        engine.toggle_rs485();
        ui.prompt_status = engine.rs485.active.load()
                               ? (engine.rs485.get_mode_name() + " STARTED.")
                               : "RS485 test STOPPED.";
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '8' || ch == 'u' || ch == 'U') {
        engine.toggle_usb_stress();
        ui.prompt_status = engine.usb_tester.active.load() ? "4-Port USB storage 4MB O_DIRECT stress STARTED." : "USB storage stress STOPPED.";
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == '9') {
        engine.toggle_can();
        ui.prompt_status = engine.can_tester.active.load()
                               ? (std::string(engine.can_tester.fd_mode.load() ? "CAN-FD 64B (" : "CAN (") +
                                  engine.can_tester.get_mode_name() + ") STARTED.")
                               : "CAN test STOPPED.";
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 'f' || ch == 'F') {
        engine.can_tester.toggle_fd_mode();
        ui.prompt_status = engine.can_tester.fd_mode.load()
                               ? "CAN switched to CAN-FD 64B @ 2Mbps (500fps high-rate stress)."
                               : "CAN switched to Standard CAN 2.0 8B @ 500kbps (50fps).";
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 'z' || ch == 'Z') {
        engine.rs232.cycle_direction();
        ui.prompt_status = "RS232 direction switched: " + engine.rs232.get_mode_name();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 'x' || ch == 'X') {
        engine.rs485.cycle_direction();
        ui.prompt_status = "RS485 direction switched: " + engine.rs485.get_mode_name();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 'd' || ch == 'D') {
        engine.can_tester.cycle_direction();
        ui.prompt_status = "CAN direction switched: " + engine.can_tester.get_mode_name();
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 'i' || ch == 'I') {
        engine.setup_default_network();
        ui.prompt_status = "4-LAN static IPs & raw sockets re-initialized (.10.1/.11.1/.20.1/.21.1).";
    } else if (ch == 'p' || ch == 'P') {
        engine.toggle_governor();
        ui.prompt_status = engine.governor_perf
                               ? "Governor locked to PERFORMANCE mode."
                               : "Governor restored to SCHEDUTIL (dynamic) mode.";
        if (!ui.config_path.empty()) save_engine_config(engine, ui.config_path);
    } else if (ch == 'r' || ch == 'R') {
        engine.reset_all_stats();
        ui.prompt_status = "All statistics (DDR, LAN, Serial, CAN, USB) reset to zero.";
    }
    return false;
}

void draw_menu(const RkTesterEngine& engine, const UiState& ui, int h, int w) {
    const int box_w = 72;
    const int box_h = 21;
    int my = std::max(1, (h - box_h) / 2);
    int mx = std::max(2, (w - box_w) / 2);

    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(my, mx, "+----------------------------------------------------------------------+");
    mvprintw(my + 1, mx, "|               TEST ITEMS CONFIGURATION & CHECKLIST                   |");
    mvprintw(my + 2, mx, "+----------------------------------------------------------------------+");
    attroff(COLOR_PAIR(4) | A_BOLD);

    attron(COLOR_PAIR(7));
    mvprintw(my + 3, mx, "| [Up/Down][Space] toggle, [0-9,V,B] item, [T]Burst [F]FD [Z/X/D] Dir: |");
    mvprintw(my + 4, mx, "|                                                                      |");
    attroff(COLOR_PAIR(7));

    struct MenuItem {
        std::string key_str;
        std::string name;
        std::string desc;
        bool checked;
    };

    std::string lan_desc = engine.net_emc.high_load.load() ? "BURST 1480B Gigabit [T]" : "EMC 100ms Mutual [T]";
    MenuItem items[11] = {
        {"[1]", "CPU 8-Core Compute Stress", "8-Core NEON+DDR burn-in", ui.menu_cpu},
        {"[2]", "Mali-G52 GPU EGL/GLES", "Panfrost FBO shader burn", ui.menu_gpu},
        {"[3]", "RK-NPU Dual-Core AI Stress", "Dual-Core 6 TOPS MatMul", ui.menu_npu},
        {"[0]", "RK-RGA Dual-Core 2D Stress", "Dual-RGA2 4K CSC+Rotate", ui.menu_rga},
        {"[V]", "RK-MPP 4K Video Stress", "Dual-4K H.265 VENC+VDEC", ui.menu_mpp},
        {"[B]", "DDR Memory Stress", "Large-RAM NEON bit-flip", ui.menu_ddr},
        {"[5]", "4-LAN Dual-Pair Ethernet", lan_desc, ui.menu_emc},
        {"[6]", "RS232 (" + engine.rs232.get_short_mode_name() + ")", engine.rs232.get_menu_desc(), ui.menu_rs232},
        {"[7]", "RS485 (" + engine.rs485.get_short_mode_name() + ")", engine.rs485.get_menu_desc(), ui.menu_rs485},
        {"[8]", "4-Port USB Storage Stress", "sda..sdd 4MB O_DIRECT", ui.menu_usb},
        {"[9]", "CAN Bus (" + engine.can_tester.get_short_mode_name() + ")", engine.can_tester.get_menu_desc(), ui.menu_can}
    };

    for (int i = 0; i < 11; ++i) {
        int cur_y = my + 5 + i;
        attron(COLOR_PAIR(4) | A_BOLD);
        mvprintw(cur_y, mx, "|");
        mvprintw(cur_y, mx + box_w - 1, "|");
        attroff(COLOR_PAIR(4) | A_BOLD);

        bool is_sel = (ui.menu_cursor == i);
        if (is_sel) {
            attron(COLOR_PAIR(2) | A_BOLD);
            mvprintw(cur_y, mx + 2, "->");
            attroff(COLOR_PAIR(2) | A_BOLD);
        } else {
            mvprintw(cur_y, mx + 2, "  ");
        }

        if (items[i].checked) {
            attron(COLOR_PAIR(1) | A_BOLD);
            mvprintw(cur_y, mx + 5, "[X]");
            attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(8) | A_BOLD);
            mvprintw(cur_y, mx + 5, "[ ]");
            attroff(COLOR_PAIR(8) | A_BOLD);
        }

        attron(COLOR_PAIR(6) | A_BOLD);
        mvprintw(cur_y, mx + 9, "%s", items[i].key_str.c_str());
        attroff(COLOR_PAIR(6) | A_BOLD);

        int text_col = is_sel ? (COLOR_PAIR(7) | A_BOLD | A_UNDERLINE) : COLOR_PAIR(7);
        attron(text_col);
        mvprintw(cur_y, mx + 13, " %-27.27s", items[i].name.c_str());
        attroff(text_col);

        // 严格限宽，确保永不溢出覆盖右边框 '|'
        attron(COLOR_PAIR(8));
        mvprintw(cur_y, mx + 42, "(%.26s)", items[i].desc.c_str());
        attroff(COLOR_PAIR(8));
    }

    int py = my + 16;
    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(py, mx, "|");
    mvprintw(py, mx + box_w - 1, "|");
    attroff(COLOR_PAIR(4) | A_BOLD);

    attron(COLOR_PAIR(7) | A_BOLD);
    mvprintw(py, mx + 3, "LAN Ports Filter (LAN 0..3):");
    attroff(COLOR_PAIR(7) | A_BOLD);

    const char* p_names[4] = {"lan0", "lan1", "lan2", "lan3"};
    int px = mx + 36;
    for (int i = 0; i < 4; ++i) {
        bool is_sel = (ui.menu_cursor == 11 + i);
        if (is_sel) {
            attron(COLOR_PAIR(2) | A_BOLD);
            mvprintw(py, px - 1, ">");
            attroff(COLOR_PAIR(2) | A_BOLD);
        }
        if (ui.menu_ports[i]) {
            attron(COLOR_PAIR(1) | A_BOLD);
            mvprintw(py, px, "[X]%s", p_names[i]);
            attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(8) | A_BOLD);
            mvprintw(py, px, "[ ]%s", p_names[i]);
            attroff(COLOR_PAIR(8) | A_BOLD);
        }
        px += 8;
    }

    int ay = my + 17;
    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(ay, mx, "|");
    mvprintw(ay, mx + box_w - 1, "|");
    attroff(COLOR_PAIR(4) | A_BOLD);

    int apply_attr = (ui.menu_cursor == 15) ? (COLOR_PAIR(1) | A_REVERSE | A_BOLD) : (COLOR_PAIR(1) | A_BOLD);
    attron(apply_attr);
    mvprintw(ay, mx + 5, "[ S / ENTER: Apply & Run Selected Tests ]");
    attroff(apply_attr);

    int cancel_attr = (ui.menu_cursor == 16) ? (COLOR_PAIR(3) | A_REVERSE | A_BOLD) : COLOR_PAIR(7);
    attron(cancel_attr);
    mvprintw(ay, mx + 49, "[ ESC: Cancel ]");
    attroff(cancel_attr);

    int hy = my + 18;
    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(hy, mx, "|");
    mvprintw(hy, mx + box_w - 1, "|");
    attroff(COLOR_PAIR(4) | A_BOLD);
    attron(COLOR_PAIR(7)); mvprintw(hy, mx + 3, "Hotkeys: "); attroff(COLOR_PAIR(7));
    attron(COLOR_PAIR(2) | A_BOLD); mvprintw(hy, mx + 12, "[A]"); attroff(COLOR_PAIR(2) | A_BOLD);
    attron(COLOR_PAIR(7)); mvprintw(hy, mx + 15, "Check All  "); attroff(COLOR_PAIR(7));
    attron(COLOR_PAIR(2) | A_BOLD); mvprintw(hy, mx + 26, "[N]"); attroff(COLOR_PAIR(2) | A_BOLD);
    attron(COLOR_PAIR(7)); mvprintw(hy, mx + 29, "Clear All  "); attroff(COLOR_PAIR(7));
    attron(COLOR_PAIR(2) | A_BOLD); mvprintw(hy, mx + 40, "[S]"); attroff(COLOR_PAIR(2) | A_BOLD);
    attron(COLOR_PAIR(7)); mvprintw(hy, mx + 43, "Apply  "); attroff(COLOR_PAIR(7));
    attron(COLOR_PAIR(2) | A_BOLD); mvprintw(hy, mx + 50, "[ESC/M]"); attroff(COLOR_PAIR(2) | A_BOLD);
    attron(COLOR_PAIR(7)); mvprintw(hy, mx + 57, "Dashboard"); attroff(COLOR_PAIR(7));

    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(my + 19, mx, "+----------------------------------------------------------------------+");
    attroff(COLOR_PAIR(4) | A_BOLD);

    attron(COLOR_PAIR(7) | A_BOLD);
    mvprintw(my + 20, mx, "Status: %-62.62s", ui.prompt_status.c_str());
    attroff(COLOR_PAIR(7) | A_BOLD);
}

std::string format_compact_count(uint64_t v) {
    char buf[24];
    if (v < 10000) {
        snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(v));
    } else if (v < 1000000) {
        snprintf(buf, sizeof(buf), "%.1fk", v / 1000.0);
    } else {
        snprintf(buf, sizeof(buf), "%.1fM", v / 1000000.0);
    }
    return buf;
}

std::string format_compact_bytes(uint64_t b) {
    char buf[24];
    if (b < 1024) {
        snprintf(buf, sizeof(buf), "%lluB", static_cast<unsigned long long>(b));
    } else if (b < 100 * 1024) {
        snprintf(buf, sizeof(buf), "%.1fK", b / 1024.0);
    } else if (b < 1024 * 1024) {
        snprintf(buf, sizeof(buf), "%.0fK", b / 1024.0);
    } else {
        snprintf(buf, sizeof(buf), "%.1fM", b / (1024.0 * 1024.0));
    }
    return buf;
}

void draw_dashboard(const RkTesterEngine& engine, const UiState& ui, int h, int w) {
    // Top title (高对比度亮青粗体 + 默认背景，避免灰字贴白底发糊)
    attron(COLOR_PAIR(14) | A_BOLD);
    mvprintw(0, 2, "RK3576 INDUSTRIAL HARDWARE TEST PLATFORM");
    attroff(COLOR_PAIR(14) | A_BOLD);

    char meta[128];
    snprintf(meta, sizeof(meta), "Uptime: %s | Gov: %s",
             ui.uptime.c_str(), engine.governor_perf ? "PERF" : "DYNAMIC");
    attron(COLOR_PAIR(7));
    mvprintw(0, std::max(46, w - static_cast<int>(std::strlen(meta)) - 3), "%s", meta);
    attroff(COLOR_PAIR(7));

    bool spacious = (h >= 32);
    bool extra_spacious = (h >= 36);
    int line = 1;
    if (spacious) {
        attron(COLOR_PAIR(4));
        mvhline(1, 2, '=', w - 4);
        attroff(COLOR_PAIR(4));
        line = 2;
    }

    double t_soc = get_thermal_val(ui.thermals, "soc");
    double t_big = get_thermal_val(ui.thermals, "bigcore");
    double t_gpu = get_thermal_val(ui.thermals, "gpu");
    double t_npu = get_thermal_val(ui.thermals, "npu");

    // ========================================================================
    // SECTION 1: COMPUTE & SOC ACCELERATORS
    // ========================================================================
    draw_section_banner(line++, w, 9, 4, "[1-4,0,V,B] COMPUTE & SOC/DDR", "CPU/GPU/NPU/RGA/MPP/DDR/Temp");

    bool ddr_on = engine.ddr_tester.active.load();
    uint64_t ddr_alloc_mb = engine.ddr_tester.get_alloc_mb();
    uint64_t ddr_bytes = engine.ddr_tester.get_bytes_tested();
    uint64_t ddr_passes = engine.ddr_tester.get_passes();
    uint64_t ddr_errs = engine.ddr_tester.get_bit_errors();
    double ddr_bw = engine.ddr_tester.get_bw_gb_s();
    double ddr_gb = static_cast<double>(ddr_bytes) / (1024.0 * 1024.0 * 1024.0);

    if (spacious) {
        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "CPU Stress:"); attroff(COLOR_PAIR(7) | A_BOLD);
        int cpu_col = engine.cpu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        int burn_n = engine.get_active_cpu_burn_cores();
        char cpu_tag[32];
        if (!engine.cpu_stress.load()) {
            snprintf(cpu_tag, sizeof(cpu_tag), "[IDLE]");
        } else if (engine.keep_isolcpus && burn_n < engine.num_cores) {
            snprintf(cpu_tag, sizeof(cpu_tag), "[STRESS ON: %dC RT]", burn_n);
        } else {
            snprintf(cpu_tag, sizeof(cpu_tag), "[STRESS ON: %d-CORES]", burn_n);
        }
        attron(cpu_col);
        mvprintw(line, 16, "%s", cpu_tag);
        attroff(cpu_col);

        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 39, "Total Load: "); attroff(COLOR_PAIR(7) | A_BOLD);
        draw_bar(line, 51, w - 54, ui.total_cpu, get_color_by_load(ui.total_cpu));
        line++;

        int half_w = (w - 4) / 2;
        int bar_w = half_w - 20;
        for (int i = 0; i < 4; ++i) {
            int f_a53 = (i < static_cast<int>(ui.cpu_freqs.size())) ? ui.cpu_freqs[i] : 0;
            double p_a53 = (i < static_cast<int>(ui.core_cpus.size())) ? ui.core_cpus[i] : 0.0;
            attron(COLOR_PAIR(7)); mvprintw(line + i, 3, "A53 Core C%d:", i); attroff(COLOR_PAIR(7));
            draw_bar(line + i, 15, bar_w, p_a53, get_color_by_load(p_a53));
            int f_a53_col = ui.throttle.cpu_little ? (COLOR_PAIR(2) | A_BOLD) : COLOR_PAIR(6);
            attron(f_a53_col); mvprintw(line + i, 15 + bar_w + 1, "%4dM", f_a53); attroff(f_a53_col);

            int bi = i + 4;
            int f_a72 = (bi < static_cast<int>(ui.cpu_freqs.size())) ? ui.cpu_freqs[bi] : 0;
            double p_a72 = (bi < static_cast<int>(ui.core_cpus.size())) ? ui.core_cpus[bi] : 0.0;
            attron(COLOR_PAIR(7)); mvprintw(line + i, half_w + 2, "A72 Core C%d:", bi); attroff(COLOR_PAIR(7));
            draw_bar(line + i, half_w + 14, bar_w, p_a72, get_color_by_load(p_a72));
            int f_a72_col = ui.throttle.cpu_big ? (COLOR_PAIR(2) | A_BOLD) : COLOR_PAIR(6);
            attron(f_a72_col); mvprintw(line + i, half_w + 14 + bar_w + 1, "%4dM", f_a72); attroff(f_a72_col);
        }
        line += 4;

        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "Mali GPU:"); attroff(COLOR_PAIR(7) | A_BOLD);
        int gpu_col = engine.gpu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(gpu_col); mvprintw(line, 13, "%s", engine.gpu_stress.load() ? "[RUN]" : "[IDLE]"); attroff(gpu_col);
        draw_bar(line, 20, 14, ui.gpu_load, get_color_by_load(ui.gpu_load));
        int gf_col = ui.throttle.gpu ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(6);
        attron(gf_col);
        if (ui.throttle.gpu) mvprintw(line, 35, "%3dM!", ui.gpu_freq);
        else mvprintw(line, 35, "%4dM", ui.gpu_freq);
        attroff(gf_col);

        attron(COLOR_PAIR(7)); mvprintw(line, 41, "| NPU:"); attroff(COLOR_PAIR(7));
        int npu_col = engine.npu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(npu_col); mvprintw(line, 48, "%s", engine.npu_stress.load() ? "[RUN]" : "[IDLE]"); attroff(npu_col);
        attron(COLOR_PAIR(7)); mvprintw(line, 55, "C0:%3.0f%% C1:%3.0f%%", ui.npu_loads[0], ui.npu_loads[1]); attroff(COLOR_PAIR(7));
        int nf_col = ui.throttle.npu ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(6);
        attron(nf_col);
        if (ui.throttle.npu) mvprintw(line, 71, "%3dM!", ui.npu_freq);
        else mvprintw(line, 71, "%4dM", ui.npu_freq);
        attroff(nf_col);
        line++;

        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "RGA2 2D:"); attroff(COLOR_PAIR(7) | A_BOLD);
        int rga_col = engine.rga_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(rga_col); mvprintw(line, 13, "%s", engine.rga_stress.load() ? "[RUN]" : "[IDLE]"); attroff(rga_col);
        attron(COLOR_PAIR(7)); mvprintw(line, 20, "C0:%3.0f%% C1:%3.0f%%", ui.rga_loads[0], ui.rga_loads[1]); attroff(COLOR_PAIR(7));
        attron(COLOR_PAIR(6)); mvprintw(line, 35, "%4dM", ui.rga_freq); attroff(COLOR_PAIR(6));

        attron(COLOR_PAIR(7)); mvprintw(line, 41, "| MPP:"); attroff(COLOR_PAIR(7));
        int mpp_col = engine.mpp_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(mpp_col); mvprintw(line, 48, "%s", engine.mpp_stress.load() ? "[RUN]" : "[IDLE]"); attroff(mpp_col);
        attron(COLOR_PAIR(7)); mvprintw(line, 55, "E:%3.0f D:%3.0ffps", ui.mpp_enc_fps, ui.mpp_dec_fps); attroff(COLOR_PAIR(7));
        attron(COLOR_PAIR(6)); mvprintw(line, 71, "%4dM", ui.mpp_freq); attroff(COLOR_PAIR(6));
        line++;

        if (extra_spacious) {
            attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "DDR Mem:"); attroff(COLOR_PAIR(7) | A_BOLD);
            int ddr_col = ddr_on ? ((ddr_errs == 0) ? (COLOR_PAIR(3) | A_BOLD) : (COLOR_PAIR(3) | A_REVERSE | A_BOLD))
                                 : COLOR_PAIR(1);
            attron(ddr_col); mvprintw(line, 13, "%s", ddr_on ? "[RUN]" : "[IDLE]"); attroff(ddr_col);
            if (ddr_on || ddr_bytes > 0) {
                attron(COLOR_PAIR(6) | A_BOLD);
                mvprintw(line, 20, "%4lluMB  BW:%4.2fGB/s", static_cast<unsigned long long>(ddr_alloc_mb), ddr_bw);
                attroff(COLOR_PAIR(6) | A_BOLD);
                attron(COLOR_PAIR(7));
                mvprintw(line, 41, "| Tested:%5.1fG Pass:%-3llu", ddr_gb, static_cast<unsigned long long>(ddr_passes));
                attroff(COLOR_PAIR(7));
                int derr_col = (ddr_errs == 0) ? (COLOR_PAIR(1) | A_BOLD) : (COLOR_PAIR(3) | A_BOLD);
                attron(derr_col);
                mvprintw(line, 66, "Err:%s", format_compact_count(ddr_errs).c_str());
                attroff(derr_col);
            } else {
                attron(COLOR_PAIR(8));
                mvprintw(line, 20, "Press [B] for large-RAM NEON bandwidth & bit-flip verify");
                attroff(COLOR_PAIR(8));
            }
            line++;
        }

        if (!extra_spacious && ddr_on) {
            attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "RAM/DDR:"); attroff(COLOR_PAIR(7) | A_BOLD);
            attron(COLOR_PAIR(7)); mvprintw(line, 12, "%3.1f/%3.1fG", ui.used_ram, ui.tot_ram); attroff(COLOR_PAIR(7));
            int dcol = (ddr_errs == 0) ? (COLOR_PAIR(6) | A_BOLD) : (COLOR_PAIR(3) | A_BOLD);
            attron(dcol);
            mvprintw(line, 21, "%4lluM %4.1fG/s %sE",
                     static_cast<unsigned long long>(ddr_alloc_mb), ddr_bw,
                     format_compact_count(ddr_errs).c_str());
            attroff(dcol);
        } else {
            attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "RAM Load:"); attroff(COLOR_PAIR(7) | A_BOLD);
            attron(COLOR_PAIR(7)); mvprintw(line, 13, "%3.1fG/%3.1fG", ui.used_ram, ui.tot_ram); attroff(COLOR_PAIR(7));
            draw_bar(line, 24, 12, ui.ram_pct, get_color_by_load(ui.ram_pct));
        }
        attron(COLOR_PAIR(5) | A_BOLD);
        mvprintw(line, 40, "| Temp: SoC:%.0fC Big:%.0fC GPU:%.0fC NPU:%.0fC%s",
                 t_soc, t_big, t_gpu, t_npu,
                 (ui.throttle.any() && w >= 88) ? " [THR]" : "");
        attroff(COLOR_PAIR(5) | A_BOLD);
        line++;
    } else {
        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "CPU:"); attroff(COLOR_PAIR(7) | A_BOLD);
        int cpu_col = engine.cpu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(cpu_col); mvprintw(line, 8, "%s", engine.cpu_stress.load() ? "[STRESS ON]" : "[IDLE]"); attroff(cpu_col);
        draw_bar(line, 20, 12, ui.total_cpu, get_color_by_load(ui.total_cpu));
        if (ddr_on) {
            int dcol = (ddr_errs == 0) ? (COLOR_PAIR(6) | A_BOLD) : (COLOR_PAIR(3) | A_BOLD);
            attron(dcol);
            mvprintw(line, 33, "| DDR:%4.1fG/s %sE", ddr_bw, format_compact_count(ddr_errs).c_str());
            attroff(dcol);
        } else {
            attron(COLOR_PAIR(7)); mvprintw(line, 33, "| RAM:%3.1fG/%3.1fG", ui.used_ram, ui.tot_ram); attroff(COLOR_PAIR(7));
        }
        attron(COLOR_PAIR(5) | A_BOLD);
        mvprintw(line, 52, "| Temp:SoC:%.0fC Big:%.0fC", t_soc, t_big);
        attroff(COLOR_PAIR(5) | A_BOLD);
        line++;

        auto draw_compact_cluster = [&](const char* lbl, int offset) {
            attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "%s", lbl); attroff(COLOR_PAIR(7) | A_BOLD);
            int cx = 8;
            for (int i = 0; i < 4; ++i) {
                int ci = offset + i;
                double p = (ci < static_cast<int>(ui.core_cpus.size())) ? ui.core_cpus[ci] : 0.0;
                int f = (ci < static_cast<int>(ui.cpu_freqs.size())) ? ui.cpu_freqs[ci] : 0;
                attron(COLOR_PAIR(7)); mvprintw(line, cx, "C%d:", ci); attroff(COLOR_PAIR(7));
                attron(get_color_by_load(p) | A_BOLD); mvprintw(line, cx + 3, "%2.0f%%", p); attroff(get_color_by_load(p) | A_BOLD);
                attron(COLOR_PAIR(6)); mvprintw(line, cx + 6, "(%dM)", f); attroff(COLOR_PAIR(6));
                cx += 17;
            }
            line++;
        };
        draw_compact_cluster("A53:", 0);
        draw_compact_cluster("A72:", 4);

        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "GPU:"); attroff(COLOR_PAIR(7) | A_BOLD);
        int gpu_col = engine.gpu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(gpu_col); mvprintw(line, 8, "%s", engine.gpu_stress.load() ? "[RUN]" : "[IDLE]"); attroff(gpu_col);
        draw_bar(line, 15, 12, ui.gpu_load, get_color_by_load(ui.gpu_load));
        int gf_col = ui.throttle.gpu ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(6);
        attron(gf_col);
        if (ui.throttle.gpu) mvprintw(line, 28, "%3dM!", ui.gpu_freq);
        else mvprintw(line, 28, "%4dM", ui.gpu_freq);
        attroff(gf_col);

        attron(COLOR_PAIR(7)); mvprintw(line, 34, "| NPU:"); attroff(COLOR_PAIR(7));
        int npu_col = engine.npu_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(npu_col); mvprintw(line, 41, "%s", engine.npu_stress.load() ? "[RUN]" : "[IDLE]"); attroff(npu_col);
        attron(COLOR_PAIR(7)); mvprintw(line, 48, "C0:%2.0f%% C1:%2.0f%%", ui.npu_loads[0], ui.npu_loads[1]); attroff(COLOR_PAIR(7));
        int nf_col = ui.throttle.npu ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(6);
        attron(nf_col);
        if (ui.throttle.npu) mvprintw(line, 62, "%3dM!", ui.npu_freq);
        else mvprintw(line, 62, "%4dM", ui.npu_freq);
        attroff(nf_col);
        attron(COLOR_PAIR(5) | A_BOLD); mvprintw(line, 68, "G:%.0fC", t_gpu); attroff(COLOR_PAIR(5) | A_BOLD);
        line++;

        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "RGA:"); attroff(COLOR_PAIR(7) | A_BOLD);
        int rga_col = engine.rga_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(rga_col); mvprintw(line, 8, "%s", engine.rga_stress.load() ? "[RUN]" : "[IDLE]"); attroff(rga_col);
        attron(COLOR_PAIR(7)); mvprintw(line, 15, "0:%3.0f 1:%3.0f%%", ui.rga_loads[0], ui.rga_loads[1]); attroff(COLOR_PAIR(7));
        attron(COLOR_PAIR(6)); mvprintw(line, 28, "%4dM", ui.rga_freq); attroff(COLOR_PAIR(6));

        attron(COLOR_PAIR(7)); mvprintw(line, 34, "| MPP:"); attroff(COLOR_PAIR(7));
        int mpp_col = engine.mpp_stress.load() ? (COLOR_PAIR(3) | A_BOLD) : COLOR_PAIR(1);
        attron(mpp_col); mvprintw(line, 41, "%s", engine.mpp_stress.load() ? "[RUN]" : "[IDLE]"); attroff(mpp_col);
        attron(COLOR_PAIR(7)); mvprintw(line, 48, "E:%3.0ff D:%3.0ff", ui.mpp_enc_fps, ui.mpp_dec_fps); attroff(COLOR_PAIR(7));
        attron(COLOR_PAIR(6)); mvprintw(line, 62, "%4dM", ui.mpp_freq); attroff(COLOR_PAIR(6));
        attron(COLOR_PAIR(5) | A_BOLD); mvprintw(line, 68, "N:%.0fC", t_npu); attroff(COLOR_PAIR(5) | A_BOLD);
        line++;
    }

    if (h >= 25) line++;

    // ========================================================================
    // SECTION 2: 4-PORT GIGABIT ETHERNET & PING MONITOR
    // ========================================================================
    EmcSnapshot emc = engine.get_emc_snapshot();
    draw_section_banner(line++, w, 10, 1, "[5/T] 4-PORT GIGABIT ETHERNET",
                        emc.high_load ? "BURST 1480B Gigabit Stress [T]" : "lan0<->lan1 & lan2<->lan3 EMC [T]");

    for (size_t i = 0; i < emc.ports.size() && i < 4; ++i) {
        const auto& port = emc.ports[i];
        int row = line + static_cast<int>(i);

        attron(COLOR_PAIR(2));
        mvprintw(row, 3, "%-19s", port.ch_name.c_str());
        attroff(COLOR_PAIR(2));

        if (!port.enabled) {
            attron(COLOR_PAIR(8) | A_BOLD); mvprintw(row, 23, "[ DISABLED  ]"); attroff(COLOR_PAIR(8) | A_BOLD);
        } else if (port.carrier) {
            attron(COLOR_PAIR(1) | A_BOLD); mvprintw(row, 23, "[ UP %4dM ]", port.speed > 0 ? port.speed : 0); attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(3)); mvprintw(row, 23, "[ LINK DOWN ]"); attroff(COLOR_PAIR(3));
        }

        std::string ip_suffix = port.current_ip;
        size_t d2 = ip_suffix.rfind('.');
        if (d2 != std::string::npos && d2 > 0) {
            size_t d1 = ip_suffix.rfind('.', d2 - 1);
            if (d1 != std::string::npos) ip_suffix = ip_suffix.substr(d1);
        }

        std::string s_sent = format_compact_count(port.sent);
        std::string s_recv = format_compact_count(port.recv);
        std::string s_lost = format_compact_count(port.lost);

        attron(COLOR_PAIR(7));
        mvprintw(row, 36, "%-6s|S:%-5s R:%-5s L:",
                 ip_suffix.c_str(), s_sent.c_str(), s_recv.c_str());
        attroff(COLOR_PAIR(7));

        int p_lost_col = (port.lost == 0) ? COLOR_PAIR(1) : (COLOR_PAIR(3) | A_BOLD);
        double p_loss_pct = (port.sent > 0) ? (port.lost * 100.0 / port.sent) : 0.0;
        attron(p_lost_col);
        mvprintw(row, 59, "%-4s", s_lost.c_str());
        attroff(p_lost_col);

        attron(COLOR_PAIR(7));
        if (port.sent > 0) mvprintw(row, 63, "(%4.1f%%)", p_loss_pct);
        else mvprintw(row, 63, "( --- )");

        bool show_mbps = emc.high_load && port.carrier && (port.tx_mbps > 0.05 || port.rx_mbps > 0.05);
        if (show_mbps && w < 94) {
            attroff(COLOR_PAIR(7));
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(row, 71, "%4.0fMb", port.tx_mbps + port.rx_mbps);
            attroff(COLOR_PAIR(6) | A_BOLD);
        } else {
            if (port.recv > 0) mvprintw(row, 71, "%4.2fms", port.rtt_cur);
            else mvprintw(row, 71, " --.-ms");
            attroff(COLOR_PAIR(7));
        }

        uint64_t hw_e = port.hw_rx_err + port.hw_tx_err + port.hw_rx_drop;
        if (show_mbps && w >= 94) {
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(row, 79, "%3.0f/%3.0fMb", port.tx_mbps, port.rx_mbps);
            attroff(COLOR_PAIR(6) | A_BOLD);
            if (hw_e > 0 && w >= 100) {
                attron(COLOR_PAIR(3) | A_BOLD);
                mvprintw(row, 90, "HW:%s", format_compact_count(hw_e).c_str());
                attroff(COLOR_PAIR(3) | A_BOLD);
            }
        } else if (hw_e > 0 && w >= 88) {
            attron(COLOR_PAIR(3) | A_BOLD);
            mvprintw(row, 79, "HW:%s", format_compact_count(hw_e).c_str());
            attroff(COLOR_PAIR(3) | A_BOLD);
        }
    }
    line += 4;

    double tot_loss_pct = (emc.total_sent > 0) ? (emc.total_lost * 100.0 / emc.total_sent) : 0.0;
    std::string rating = "STANDBY";
    int rate_col = COLOR_PAIR(7);
    if (emc.total_sent > 0) {
        if (emc.total_flaps > 0) {
            rating = "CLASS C (FAIL)";
            rate_col = COLOR_PAIR(3) | A_BOLD;
        } else if (emc.total_lost > 0) {
            if (tot_loss_pct < 1.0) {
                rating = "CLASS B (<1%)";
                rate_col = COLOR_PAIR(2) | A_BOLD;
            } else {
                rating = "CLASS C (FAIL)";
                rate_col = COLOR_PAIR(3) | A_BOLD;
            }
        } else {
            rating = "CLASS A (PASS)";
            rate_col = COLOR_PAIR(1) | A_BOLD;
        }
    }

    const char* emc_st = engine.net_emc.active.load()
                             ? (emc.high_load ? "[BURST 1480B]" : "[RUN 100ms]")
                             : (emc.high_load ? "[STBY BURST]" : "[STANDBY]");
    int emc_col = engine.net_emc.active.load()
                      ? (emc.high_load ? (COLOR_PAIR(6) | A_BOLD) : (COLOR_PAIR(1) | A_BOLD))
                      : COLOR_PAIR(2);

    std::string tot_s = format_compact_count(emc.total_sent);
    std::string tot_r = format_compact_count(emc.total_recv);
    std::string tot_l = format_compact_count(emc.total_lost);

    if (spacious) {
        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "Mutual Ping:"); attroff(COLOR_PAIR(7) | A_BOLD);
        attron(emc_col); mvprintw(line, 16, "%-13s", emc_st); attroff(emc_col);
        attron(rate_col); mvprintw(line, 30, "[%s]", rating.c_str()); attroff(rate_col);
        if (emc.high_load) {
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(line, 48, "TX:%5.1f RX:%5.1f Mbps", emc.total_tx_mbps, emc.total_rx_mbps);
            attroff(COLOR_PAIR(6) | A_BOLD);
        } else {
            attron(COLOR_PAIR(7)); mvprintw(line, 48, "Pairs: lan0<->lan1 | lan2<->lan3"); attroff(COLOR_PAIR(7));
        }
        line++;

        attron(COLOR_PAIR(7));
        mvprintw(line, 3, "Ping Stats:  Sent:%-6s Recv:%-6s Lost:", tot_s.c_str(), tot_r.c_str());
        attroff(COLOR_PAIR(7));
        int lost_col = (emc.total_lost == 0) ? COLOR_PAIR(1) : (COLOR_PAIR(3) | A_BOLD);
        attron(lost_col); mvprintw(line, 45, "%-5s", tot_l.c_str()); attroff(lost_col);
        attron(COLOR_PAIR(7));
        mvprintw(line, 51, "(%4.1f%%) | Link Flaps:%-2llu", tot_loss_pct, static_cast<unsigned long long>(emc.total_flaps));
        attroff(COLOR_PAIR(7));
        line++;

        attron(COLOR_PAIR(7)); mvprintw(line, 3, "Latest Event: "); attroff(COLOR_PAIR(7));
        attron(COLOR_PAIR(emc.latest_event_color) | A_BOLD);
        int max_ev_len = std::max(0, w - 18);
        mvprintw(line, 17, "%.*s", max_ev_len, emc.latest_event.c_str());
        attroff(COLOR_PAIR(emc.latest_event_color) | A_BOLD);
        line++;
    } else {
        attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "Ping:"); attroff(COLOR_PAIR(7) | A_BOLD);
        attron(emc_col); mvprintw(line, 9, "%-13s", emc_st); attroff(emc_col);
        attron(rate_col); mvprintw(line, 23, "[%s]", rating.c_str()); attroff(rate_col);
        if (emc.high_load) {
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(line, 40, "TX:%.0f/RX:%.0fMb ", emc.total_tx_mbps, emc.total_rx_mbps);
            attroff(COLOR_PAIR(6) | A_BOLD);
            attron(COLOR_PAIR(7)); mvprintw(line, 57, "L:"); attroff(COLOR_PAIR(7));
            int lost_col = (emc.total_lost == 0) ? COLOR_PAIR(1) : (COLOR_PAIR(3) | A_BOLD);
            attron(lost_col); mvprintw(line, 59, "%-4s", tot_l.c_str()); attroff(lost_col);
            attron(COLOR_PAIR(7)); mvprintw(line, 64, "F:%-2llu", static_cast<unsigned long long>(emc.total_flaps)); attroff(COLOR_PAIR(7));
        } else {
            attron(COLOR_PAIR(7));
            mvprintw(line, 40, "S:%-5s L:", tot_s.c_str());
            attroff(COLOR_PAIR(7));
            int lost_col = (emc.total_lost == 0) ? COLOR_PAIR(1) : (COLOR_PAIR(3) | A_BOLD);
            attron(lost_col); mvprintw(line, 50, "%-4s", tot_l.c_str()); attroff(lost_col);
            attron(COLOR_PAIR(7)); mvprintw(line, 55, "F:%-2llu", static_cast<unsigned long long>(emc.total_flaps)); attroff(COLOR_PAIR(7));
        }
        line++;
    }

    if (h >= 26) line++;

    // ========================================================================
    // SECTION 3: INDUSTRIAL FIELDBUS (SERIAL & CAN)
    // ========================================================================
    draw_section_banner(line++, w, 11, 2, "[6,7,9/F] INDUSTRIAL FIELDBUS", "Dir:[Z]232 [X]485 [D]CAN | [F]CAN-FD");

    auto draw_serial_row = [&](const SerialChannel& sc) {
        std::string short_lbl = sc.get_short_label();
        std::string wide_lbl = sc.get_wide_label();
        bool on = sc.active.load();
        ChannelState st = sc.state.load();
        uint64_t txf = sc.tx_frames.load();
        uint64_t rxf = sc.rx_frames.load();
        uint64_t errs = sc.err_frames.load();
        uint64_t crcs = sc.crc_errors.load();
        uint64_t txb = sc.tx_bytes.load();
        uint64_t rxb = sc.rx_bytes.load();
        double spd = sc.speed_kb.load();
        double tfps = sc.tx_fps.load();
        double rfps = sc.rx_fps.load();
        double rtt = sc.rtt_cur_ms.load();
        double rtt_max = sc.rtt_max_ms.load();
        uint16_t seq = sc.last_rx_seq.load();
        double loss_pct = (txf > 0) ? (errs * 100.0 / txf) : 0.0;

        uint64_t now_us = get_time_us();
        uint64_t last_rx_us = sc.last_valid_rx_us.load();
        bool comm_ok = on && (last_rx_us > 0) && (now_us >= last_rx_us) && ((now_us - last_rx_us) <= 600000);
        bool comm_lost = on && !comm_ok && (errs > 0 || (txf >= 25 && rxf == 0));

        const char* st_tag = "[IDLE]";
        const char* res_tag = "[IDLE]";
        int st_col = COLOR_PAIR(7);
        int res_col = COLOR_PAIR(7);
        if (st == ChannelState::PORT_ERROR) {
            st_tag = "[ERR] ";
            res_tag = "[ERR] ";
            st_col = COLOR_PAIR(3) | A_BOLD;
            res_col = COLOR_PAIR(3) | A_BOLD;
        } else if (on) {
            st_tag = "[RUN] ";
            if (comm_ok) {
                // 正常通信中：无论历史是否出现过错误，实时状态始终显示 [COMM]（错误次数在 E:/CRC: 如实累计）
                st_col = COLOR_PAIR(1) | A_BOLD;
                res_tag = "[COMM]";
                res_col = COLOR_PAIR(1) | A_BOLD;
            } else if (comm_lost) {
                st_col = COLOR_PAIR(2) | A_BOLD;
                res_tag = "[NORX]";
                res_col = COLOR_PAIR(3) | A_BOLD;
            } else {
                st_col = COLOR_PAIR(2) | A_BOLD;
                res_tag = "[WAIT]";
                res_col = COLOR_PAIR(2) | A_BOLD;
            }
        }

        if (st == ChannelState::PORT_ERROR) {
            attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "%-13s", short_lbl.c_str()); attroff(COLOR_PAIR(7) | A_BOLD);
            attron(st_col); mvprintw(line, 17, "%s", st_tag); attroff(st_col);
            std::string err_msg = "OPEN FAIL: " + sc.get_error_detail();
            int max_len = std::max(10, w - 26);
            attron(COLOR_PAIR(3)); mvprintw(line, 24, "%.*s", max_len, err_msg.c_str()); attroff(COLOR_PAIR(3));
            line++;
            if (extra_spacious) {
                std::string hint = "- " + sc.get_mode_name() + " (check /dev/ttyS* node & permissions)";
                attron(COLOR_PAIR(8)); mvprintw(line, 5, "%.*s", std::max(10, w - 7), hint.c_str()); attroff(COLOR_PAIR(8));
                line++;
            }
            return;
        }

        int err_col = (errs == 0 && crcs == 0) ? (on ? COLOR_PAIR(1) : COLOR_PAIR(7)) : (COLOR_PAIR(3) | A_BOLD);
        std::string s_tx = format_compact_count(txf);
        std::string s_rx = format_compact_count(rxf);
        std::string s_err = format_compact_count(errs);
        std::string s_txb = format_compact_bytes(txb);
        std::string s_rxb = format_compact_bytes(rxb);

        if (w >= 104) {
            attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "%-14s", wide_lbl.c_str()); attroff(COLOR_PAIR(7) | A_BOLD);
            attron(st_col); mvprintw(line, 18, "%s", st_tag); attroff(st_col);
            attron(COLOR_PAIR(7));
            mvprintw(line, 25, "TX:%-5s(%-5s) RX:%-5s(%-5s)",
                     s_tx.c_str(), s_txb.c_str(), s_rx.c_str(), s_rxb.c_str());
            attroff(COLOR_PAIR(7));
            attron(err_col); mvprintw(line, 57, "Err:%-3s", s_err.c_str()); attroff(err_col);
            attron(COLOR_PAIR(7));
            if (txf > 0) mvprintw(line, 64, "(%4.1f%%)", loss_pct);
            else mvprintw(line, 64, "( --- )");
            mvprintw(line, 72, "CRC:%-2llu", static_cast<unsigned long long>(crcs));
            attroff(COLOR_PAIR(7));
            attron(COLOR_PAIR(6) | A_BOLD); mvprintw(line, 79, "%4.1fK/s", spd); attroff(COLOR_PAIR(6) | A_BOLD);
            attron(COLOR_PAIR(4));
            if ((on && comm_ok) || (!on && rxf > 0)) mvprintw(line, 87, "%4.1fms", rtt);
            else mvprintw(line, 87, " --.-ms");
            attroff(COLOR_PAIR(4));
            attron(res_col); mvprintw(line, std::min(96, w - 7), "%s", res_tag); attroff(res_col);
        } else {
            attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "%-13s", short_lbl.c_str()); attroff(COLOR_PAIR(7) | A_BOLD);
            attron(st_col); mvprintw(line, 17, "%s", st_tag); attroff(st_col);
            attron(COLOR_PAIR(7));
            mvprintw(line, 24, "T:%-4s R:%-4s", s_tx.c_str(), s_rx.c_str());
            attroff(COLOR_PAIR(7));
            attron(err_col); mvprintw(line, 38, "E:%-3s", s_err.c_str()); attroff(err_col);
            attron(COLOR_PAIR(7));
            if (txf > 0) mvprintw(line, 44, "(%4.1f%%)", loss_pct);
            else mvprintw(line, 44, "( --- )");
            attroff(COLOR_PAIR(7));
            attron(COLOR_PAIR(6) | A_BOLD); mvprintw(line, 52, "%4.1fK/s", spd); attroff(COLOR_PAIR(6) | A_BOLD);
            attron(COLOR_PAIR(4));
            if ((on && comm_ok) || (!on && rxf > 0)) mvprintw(line, 60, "%4.1fms", rtt);
            else mvprintw(line, 60, " --.-ms");
            attroff(COLOR_PAIR(4));
            attron(res_col); mvprintw(line, std::min(69, w - 7), "%s", res_tag); attroff(res_col);
        }
        line++;

        if (extra_spacious) {
            const char* mode_tag = (sc.dir.load() == SerialDirection::PING_PONG) ? "PP" : "Uni";
            uint64_t hw_oe = sc.hw_oe.load();
            uint64_t hw_fe = sc.hw_fe.load();
            uint64_t hw_pe = sc.hw_pe.load();
            bool has_hw_err = (hw_oe > 0 || hw_fe > 0 || hw_pe > 0);
            char detail[192];
            if (rxf > 0) {
                if (has_hw_err) {
                    snprintf(detail, sizeof(detail),
                             "- 115200 8N1 64B %s | TX:%s(%.0fF/s) RX:%s(%.0fF/s) HW:oe%llu/fe%llu/pe%llu Max:%.1fms",
                             mode_tag, s_txb.c_str(), tfps, s_rxb.c_str(), rfps,
                             static_cast<unsigned long long>(hw_oe),
                             static_cast<unsigned long long>(hw_fe),
                             static_cast<unsigned long long>(hw_pe), rtt_max);
                } else {
                    snprintf(detail, sizeof(detail),
                             "- 115200 8N1 64B %s | TX:%s(%.0fF/s) RX:%s(%.0fF/s) CRC:%llu Seq:#%u Max:%.1fms",
                             mode_tag, s_txb.c_str(), tfps, s_rxb.c_str(), rfps,
                             static_cast<unsigned long long>(crcs), static_cast<unsigned>(seq), rtt_max);
                }
            } else {
                snprintf(detail, sizeof(detail),
                         "- 115200 8N1 64B | %s | TX:%s(%.0fF/s) RX:%s(%.0fF/s) CRC:%llu HW:oe%llu/fe%llu",
                         sc.get_short_mode_name().c_str(),
                         s_txb.c_str(), tfps, s_rxb.c_str(), rfps,
                         static_cast<unsigned long long>(crcs),
                         static_cast<unsigned long long>(hw_oe),
                         static_cast<unsigned long long>(hw_fe));
            }
            int dcol = has_hw_err ? (COLOR_PAIR(2) | A_BOLD) : COLOR_PAIR(7);
            attron(dcol);
            mvprintw(line, 5, "%.*s", std::max(10, w - 7), detail);
            attroff(dcol);
            line++;
        }
    };

    draw_serial_row(engine.rs232);
    draw_serial_row(engine.rs485);

    // CAN row (与 RS232/RS485 完全对齐的高信息密度展示)
    {
        const auto& can = engine.can_tester;
        std::string short_lbl = can.get_short_label();
        std::string wide_lbl = can.get_wide_label();
        bool can_on = can.active.load();
        ChannelState can_state = can.state.load();
        uint64_t c_tx = can.tx_frames.load();
        uint64_t c_rx = can.rx_frames.load();
        uint64_t c_err = can.err_frames.load();
        uint64_t c_crc = can.crc_errors.load();
        uint64_t c_bus = can.bus_errors.load();
        uint64_t c_txb = can.tx_bytes.load();
        uint64_t c_rxb = can.rx_bytes.load();
        double c_tx_fps = can.tx_fps.load();
        double c_rx_fps = can.rx_fps.load();
        double c_rtt = can.rtt_cur_ms.load();
        double c_rtt_max = can.rtt_max_ms.load();
        uint8_t c_seq = can.last_rx_seq.load();
        CanDirection c_dir = can.dir.load();
        double c_loss_pct = (c_tx > 0) ? (c_err * 100.0 / c_tx) : 0.0;
        bool has_any_err = (c_err > 0 || c_crc > 0 || c_bus > 0);

        uint64_t now_us = get_time_us();
        uint64_t last_rx_us = can.last_valid_rx_us.load();
        bool can_comm_ok = can_on && (last_rx_us > 0) && (now_us >= last_rx_us) && ((now_us - last_rx_us) <= 600000);
        bool can_comm_lost = can_on && !can_comm_ok && (c_err > 0 || (c_tx >= 25 && c_rx == 0));

        const char* st_tag = "[IDLE]";
        const char* res_tag = "[IDLE]";
        int st_col = COLOR_PAIR(7);
        int res_col = COLOR_PAIR(7);
        if (can_state == ChannelState::PORT_ERROR) {
            st_tag = "[ERR] ";
            res_tag = "[ERR] ";
            st_col = COLOR_PAIR(3) | A_BOLD;
            res_col = COLOR_PAIR(3) | A_BOLD;
        } else if (can_on) {
            st_tag = "[RUN] ";
            if (can_comm_ok) {
                st_col = COLOR_PAIR(1) | A_BOLD;
                res_tag = "[COMM]";
                res_col = COLOR_PAIR(1) | A_BOLD;
            } else if (can_comm_lost) {
                st_col = COLOR_PAIR(2) | A_BOLD;
                res_tag = "[NORX]";
                res_col = COLOR_PAIR(3) | A_BOLD;
            } else {
                st_col = COLOR_PAIR(2) | A_BOLD;
                res_tag = "[WAIT]";
                res_col = COLOR_PAIR(2) | A_BOLD;
            }
        }

        if (can_state == ChannelState::PORT_ERROR) {
            attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "%-13s", short_lbl.c_str()); attroff(COLOR_PAIR(7) | A_BOLD);
            attron(st_col); mvprintw(line, 17, "%s", st_tag); attroff(st_col);
            std::string err_msg = "OPEN FAIL: " + can.get_error_detail();
            attron(COLOR_PAIR(3)); mvprintw(line, 24, "%.*s", std::max(10, w - 26), err_msg.c_str()); attroff(COLOR_PAIR(3));
            line++;
            if (extra_spacious) {
                std::string hint = "- " + can.get_mode_name() + " (check can0/can1 driver & ip link up)";
                attron(COLOR_PAIR(8)); mvprintw(line, 5, "%.*s", std::max(10, w - 7), hint.c_str()); attroff(COLOR_PAIR(8));
                line++;
            }
        } else {
            int err_col = (!has_any_err) ? (can_on ? COLOR_PAIR(1) : COLOR_PAIR(7)) : (COLOR_PAIR(3) | A_BOLD);
            std::string s_tx = format_compact_count(c_tx);
            std::string s_rx = format_compact_count(c_rx);
            std::string s_err = format_compact_count(c_err);
            std::string s_txb = format_compact_bytes(c_txb);
            std::string s_rxb = format_compact_bytes(c_rxb);

            if (w >= 104) {
                attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "%-14s", wide_lbl.c_str()); attroff(COLOR_PAIR(7) | A_BOLD);
                attron(st_col); mvprintw(line, 18, "%s", st_tag); attroff(st_col);
                attron(COLOR_PAIR(7));
                mvprintw(line, 25, "TX:%-5s(%-5s) RX:%-5s(%-5s)",
                         s_tx.c_str(), s_txb.c_str(), s_rx.c_str(), s_rxb.c_str());
                attroff(COLOR_PAIR(7));
                attron(err_col); mvprintw(line, 57, "Err:%-3s", s_err.c_str()); attroff(err_col);
                attron(COLOR_PAIR(7));
                if (c_tx > 0) mvprintw(line, 64, "(%4.1f%%)", c_loss_pct);
                else mvprintw(line, 64, "( --- )");
                mvprintw(line, 72, "Bus:%-2llu", static_cast<unsigned long long>(c_bus));
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(6) | A_BOLD); mvprintw(line, 79, "%3.0fF/s", c_rx_fps); attroff(COLOR_PAIR(6) | A_BOLD);
                attron(COLOR_PAIR(4));
                if ((can_on && can_comm_ok) || (!can_on && c_rx > 0)) mvprintw(line, 87, "%4.2fms", c_rtt);
                else mvprintw(line, 87, " --.-ms");
                attroff(COLOR_PAIR(4));
                attron(res_col); mvprintw(line, std::min(96, w - 7), "%s", res_tag); attroff(res_col);
            } else {
                attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "%-13s", short_lbl.c_str()); attroff(COLOR_PAIR(7) | A_BOLD);
                attron(st_col); mvprintw(line, 17, "%s", st_tag); attroff(st_col);
                attron(COLOR_PAIR(7));
                mvprintw(line, 24, "T:%-4s R:%-4s", s_tx.c_str(), s_rx.c_str());
                attroff(COLOR_PAIR(7));
                attron(err_col); mvprintw(line, 38, "E:%-3s", s_err.c_str()); attroff(err_col);
                attron(COLOR_PAIR(7));
                if (c_tx > 0) mvprintw(line, 44, "(%4.1f%%)", c_loss_pct);
                else mvprintw(line, 44, "( --- )");
                attroff(COLOR_PAIR(7));
                attron(COLOR_PAIR(6) | A_BOLD); mvprintw(line, 52, "%3.0fF/s", c_rx_fps); attroff(COLOR_PAIR(6) | A_BOLD);
                attron(COLOR_PAIR(4));
                if ((can_on && can_comm_ok) || (!can_on && c_rx > 0)) mvprintw(line, 60, "%4.2fms", c_rtt);
                else mvprintw(line, 60, " --.-ms");
                attroff(COLOR_PAIR(4));
                attron(res_col); mvprintw(line, std::min(69, w - 7), "%s", res_tag); attroff(res_col);
            }
            line++;

            if (extra_spacious) {
                const char* id_str = (c_dir == CanDirection::CAN0_TO_CAN1) ? "ID:0x121"
                                   : (c_dir == CanDirection::CAN1_TO_CAN0) ? "ID:0x221"
                                                                           : "ID:0x121<>0x221";
                const char* frame_tag = can.fd_mode.load() ? "FD64B" : "8B";
                char detail[160];
                if (c_rx > 0) {
                    snprintf(detail, sizeof(detail),
                             "- %s %s %s | TX:%.0fF/s RX:%.0fF/s | CRC:%llu Bus:%llu Seq:#%u Max:%.2fms",
                             can.get_bitrate_info().c_str(), frame_tag, id_str, c_tx_fps, c_rx_fps,
                             static_cast<unsigned long long>(c_crc), static_cast<unsigned long long>(c_bus),
                             static_cast<unsigned>(c_seq), c_rtt_max);
                } else {
                    snprintf(detail, sizeof(detail),
                             "- %s %s %s | %s | TX:%.0fF/s RX:%.0fF/s CRC:%llu Bus:%llu",
                             can.get_bitrate_info().c_str(), frame_tag, id_str, can.get_short_mode_name().c_str(),
                             c_tx_fps, c_rx_fps,
                             static_cast<unsigned long long>(c_crc), static_cast<unsigned long long>(c_bus));
                }
                attron(COLOR_PAIR(7));
                mvprintw(line, 5, "%.*s", std::max(10, w - 7), detail);
                attroff(COLOR_PAIR(7));
                line++;
            }
        }
    }

    if (h >= 27) line++;

    // ========================================================================
    // SECTION 4: 4-PORT HIGH-SPEED USB STORAGE
    // ========================================================================
    draw_section_banner(line++, w, 12, 5, "[8] 4-PORT USB STORAGE", "sda..sdd Read/Write Stress & Integrity");

    bool usb_on = engine.usb_tester.active.load();
    uint64_t usb_w_mb = engine.usb_tester.get_total_write_mb();
    uint64_t usb_r_mb = engine.usb_tester.get_total_read_mb();
    uint64_t usb_errs = engine.usb_tester.get_total_errors();
    double usb_w_spd = engine.usb_tester.get_total_write_speed();
    double usb_r_spd = engine.usb_tester.get_total_read_speed();
    int mounted_cnt = 0;
    for (const auto& d : engine.usb_tester.disks) {
        if (d->mounted.load()) ++mounted_cnt;
    }

    attron(COLOR_PAIR(7) | A_BOLD); mvprintw(line, 3, "4-Disk Stress:"); attroff(COLOR_PAIR(7) | A_BOLD);
    char usb_run_tag[24];
    int usb_col = COLOR_PAIR(2);
    if (!usb_on) {
        snprintf(usb_run_tag, sizeof(usb_run_tag), "[STANDBY]");
    } else if (mounted_cnt == 0) {
        snprintf(usb_run_tag, sizeof(usb_run_tag), "[NO DISK]");
        usb_col = (usb_errs > 0) ? (COLOR_PAIR(3) | A_BOLD) : (COLOR_PAIR(2) | A_BOLD);
    } else {
        snprintf(usb_run_tag, sizeof(usb_run_tag), "[RUN %d/4]", mounted_cnt);
        usb_col = COLOR_PAIR(1) | A_BOLD;
    }
    attron(usb_col); mvprintw(line, 18, "%s", usb_run_tag); attroff(usb_col);
    attron(COLOR_PAIR(7));
    mvprintw(line, 28, "W:%-4lluM(%2.0fM/s) R:%-4lluM(%2.0fM/s)",
             static_cast<unsigned long long>(usb_w_mb), usb_w_spd,
             static_cast<unsigned long long>(usb_r_mb), usb_r_spd);
    attroff(COLOR_PAIR(7));

    std::string s_uerr = format_compact_count(usb_errs);
    int u_err_col = (usb_errs > 0) ? (COLOR_PAIR(3) | A_BOLD)
                                   : ((usb_on && mounted_cnt > 0) ? COLOR_PAIR(1) : COLOR_PAIR(7));
    attron(u_err_col); mvprintw(line, 58, "E:%-3s", s_uerr.c_str()); attroff(u_err_col);

    const char* u_tag = "[IDLE] ";
    int u_tag_col = COLOR_PAIR(7);
    if (usb_on) {
        if (mounted_cnt == 0) {
            u_tag = "[NODEV]";
            u_tag_col = (usb_errs > 0) ? (COLOR_PAIR(3) | A_BOLD) : (COLOR_PAIR(2) | A_BOLD);
        } else if (usb_w_mb > 0 || usb_r_mb > 0 || usb_w_spd > 0.1 || usb_r_spd > 0.1) {
            u_tag = "[R/W]  ";
            u_tag_col = COLOR_PAIR(1) | A_BOLD;
        } else {
            u_tag = "[WAIT] ";
            u_tag_col = COLOR_PAIR(2) | A_BOLD;
        }
    }
    attron(u_tag_col); mvprintw(line, std::min(69, w - 8), "%s", u_tag); attroff(u_tag_col);
    line++;

    auto draw_usb_disk_inline = [&](int y, int x, const UsbDiskInfo& d, bool is_first) {
        auto dmeta = d.get_meta();
        if (!d.mounted.load()) {
            if (d.errors.load() > 0) {
                attron(COLOR_PAIR(3) | A_BOLD);
                mvprintw(y, x, "%s%-3s: [DISCONNECTED (%lluE)]",
                         is_first ? "* " : "", dmeta.dev_name.c_str(),
                         static_cast<unsigned long long>(d.errors.load()));
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else {
                attron(COLOR_PAIR(8));
                mvprintw(y, x, "%s%-3s: [UNPLUGGED / NO DEV]", is_first ? "* " : "", dmeta.dev_name.c_str());
                attroff(COLOR_PAIR(8));
            }
            return;
        }

        attron(COLOR_PAIR(2) | A_BOLD);
        mvprintw(y, x, "%s%-3s:", is_first ? "* " : "", dmeta.dev_name.c_str());
        attroff(COLOR_PAIR(2) | A_BOLD);

        int px = x + (is_first ? 6 : 4);
        uint64_t w_mb = d.write_bytes.load() / (1024 * 1024);
        uint64_t r_mb = d.read_bytes.load() / (1024 * 1024);
        double wspd = d.write_speed_mb.load();
        double rspd = d.read_speed_mb.load();
        uint64_t derr = d.errors.load();

        if (!usb_on && w_mb == 0 && r_mb == 0) {
            attron(COLOR_PAIR(1));
            std::string m = dmeta.model.empty() ? "Ready" : dmeta.model.substr(0, 15);
            mvprintw(y, px + 1, "Ready (%s)", m.c_str());
            attroff(COLOR_PAIR(1));
        } else {
            attron(COLOR_PAIR(7)); mvprintw(y, px + 1, "W:%-3lluM", static_cast<unsigned long long>(w_mb)); attroff(COLOR_PAIR(7));
            attron(COLOR_PAIR(6)); mvprintw(y, px + 7, "(%2.0fM)", wspd); attroff(COLOR_PAIR(6));
            attron(COLOR_PAIR(7)); mvprintw(y, px + 13, "R:%-3lluM", static_cast<unsigned long long>(r_mb)); attroff(COLOR_PAIR(7));
            attron(COLOR_PAIR(6)); mvprintw(y, px + 19, "(%2.0fM)", rspd); attroff(COLOR_PAIR(6));

            int tag_col = (derr > 0) ? (COLOR_PAIR(3) | A_BOLD)
                                     : (usb_on ? (COLOR_PAIR(1) | A_BOLD) : COLOR_PAIR(7));
            std::string tag = (derr > 0) ? ("[" + std::to_string(derr) + "E]")
                                         : (usb_on ? "[RW]" : "[OK]");
            attron(tag_col);
            mvprintw(y, px + 25, "%s", tag.c_str());
            attroff(tag_col);
        }
    };

    if (engine.usb_tester.disks.size() >= 2) {
        draw_usb_disk_inline(line, 3, *engine.usb_tester.disks[0], true);
        attron(COLOR_PAIR(7)); mvprintw(line, 39, "|"); attroff(COLOR_PAIR(7));
        draw_usb_disk_inline(line, 41, *engine.usb_tester.disks[1], false);
        line++;
    }
    if (engine.usb_tester.disks.size() >= 4) {
        draw_usb_disk_inline(line, 3, *engine.usb_tester.disks[2], true);
        attron(COLOR_PAIR(7)); mvprintw(line, 39, "|"); attroff(COLOR_PAIR(7));
        draw_usb_disk_inline(line, 41, *engine.usb_tester.disks[3], false);
        line++;
    }

    // Bottom Footer
    int key_y = std::max(line, h - 2);
    int status_y = std::max(line + 1, h - 1);
    int max_footer_w = std::max(10, w - 3);

    attron(COLOR_PAIR(2) | A_BOLD);
    mvprintw(key_y, 2, "%.*s", max_footer_w,
             "[Keys] [M]Menu [1-4,0,V,B]SoC/DDR [5/T]LAN [6/7]Ser [8]USB [9/F]CAN [Q]Quit");
    attroff(COLOR_PAIR(2) | A_BOLD);

    int max_status_w = std::max(10, w - 11);
    attron(COLOR_PAIR(7) | A_BOLD);
    mvprintw(status_y, 2, "Status: %-*.*s", max_status_w, max_status_w, ui.prompt_status.c_str());
    attroff(COLOR_PAIR(7) | A_BOLD);
}

} // namespace

void init_ncurses_ui() {
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    timeout(100);
    keypad(stdscr, TRUE);

    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_GREEN, -1);
        init_pair(2, COLOR_YELLOW, -1);
        init_pair(3, COLOR_RED, -1);
        init_pair(4, COLOR_CYAN, -1);
        init_pair(5, COLOR_MAGENTA, -1);
        init_pair(6, COLOR_BLUE, -1);
        init_pair(7, COLOR_WHITE, -1);
        init_pair(8, COLOR_BLACK, -1);
        init_pair(9, COLOR_CYAN, -1);     // Compute & SoC title (transparent bg)
        init_pair(10, COLOR_GREEN, -1);   // Gigabit Ethernet title (transparent bg)
        init_pair(11, COLOR_YELLOW, -1);  // Fieldbus title (transparent bg)
        init_pair(12, COLOR_MAGENTA, -1); // USB Storage title (transparent bg)
        init_pair(13, COLOR_WHITE, -1);   // Highlights
        init_pair(14, COLOR_CYAN, -1);    // Top Header title (transparent bg)
    }
}

void shutdown_ncurses_ui() {
    endwin();
}

void run_tui_loop(RkTesterEngine& engine, UiState& ui) {
    constexpr int kDrawIntervalMs = 250; // 4Hz 渲染兼顾流畅度与低 SSH/CPU 开销，按键时立即重绘
    constexpr int kHwIntervalMs = 500;
    auto last_draw = std::chrono::steady_clock::now() - std::chrono::milliseconds(kDrawIntervalMs);
    auto last_hw_update = std::chrono::steady_clock::now() - std::chrono::milliseconds(kHwIntervalMs);
    bool dirty = true;

    while (!g_quit_flag) {
        auto now = std::chrono::steady_clock::now();

        // 1. 定时采样系统硬件负载状态 (每 500ms)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_hw_update).count() >= kHwIntervalMs) {
            last_hw_update = now;
            std::string stress_warn = engine.check_stress_health();
            if (!stress_warn.empty()) ui.prompt_status = stress_warn;
            std::tie(ui.total_cpu, ui.core_cpus) = engine.get_cpu_loads();
            ui.cpu_freqs = engine.get_cpu_freqs();
            std::tie(ui.gpu_load, ui.gpu_freq) = engine.get_gpu_stats();
            std::tie(ui.npu_loads, ui.npu_freq) = engine.get_npu_stats();
            std::tie(ui.rga_loads, ui.rga_freq) = engine.get_rga_stats();
            MppStats ms = engine.get_mpp_stats();
            ui.mpp_enc_fps = ms.enc_fps;
            ui.mpp_dec_fps = ms.dec_fps;
            ui.mpp_freq = (ms.enc_freq_mhz > 0) ? ms.enc_freq_mhz : ms.dec_freq_mhz;
            std::tie(ui.used_ram, ui.tot_ram, ui.ram_pct) = engine.get_mem_stats();
            ui.thermals = engine.get_thermals();
            ui.throttle = engine.get_throttle_state();
            ui.uptime = engine.get_uptime();
            dirty = true;
        }

        // 2. 定时渲染 TUI 界面 (250ms 周期或有按键/状态变化时立即刷新)
        if (dirty || std::chrono::duration_cast<std::chrono::milliseconds>(now - last_draw).count() >= kDrawIntervalMs) {
            last_draw = now;
            dirty = false;
            erase();
            int h = 0, w = 0;
            getmaxyx(stdscr, h, w);

            if (h < 23 || w < 76) {
                attron(COLOR_PAIR(3) | A_BOLD);
                mvprintw(0, 0, "Terminal window too small! Need >= 76x23, current %dx%d", w, h);
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else if (ui.menu_open) {
                draw_menu(engine, ui, h, w);
            } else {
                draw_dashboard(engine, ui, h, w);
            }
            refresh();
        }

        // 3. 事件驱动等待用户输入（替代 15ms 忙等轮询，按键零延迟响应且极低 CPU 占用）
        auto after = std::chrono::steady_clock::now();
        int draw_rem = kDrawIntervalMs - static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(after - last_draw).count());
        int hw_rem = kHwIntervalMs - static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(after - last_hw_update).count());
        int wait_ms = std::max(10, std::min(100, std::min(draw_rem, hw_rem)));
        timeout(wait_ms);

        int ch = getch();
        if (ch != ERR) {
            if (ch != KEY_RESIZE) {
                bool quit_req = ui.menu_open
                                    ? handle_menu_input(ch, engine, ui)
                                    : handle_dashboard_input(ch, engine, ui);
                if (quit_req) break;
            }
            dirty = true;
        }
    }
}
