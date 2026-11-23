#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct UsbDiskInfo {
    size_t slot_idx{0};
    std::string mount_point;
    std::string label;

    mutable std::mutex meta_mtx;
    std::string dev_name;
    std::string part_name;
    std::string fs_type;
    std::string model;
    std::string usb_port_path;
    double capacity_gb{0.0};

    std::atomic<bool> mounted{false};
    std::atomic<uint64_t> write_bytes{0};
    std::atomic<uint64_t> read_bytes{0};
    std::atomic<uint64_t> write_ops{0};
    std::atomic<uint64_t> read_ops{0};
    std::atomic<uint64_t> errors{0};
    std::atomic<uint64_t> disconnects{0};
    std::atomic<uint64_t> recovers{0};
    std::atomic<double> write_speed_mb{0.0};
    std::atomic<double> read_speed_mb{0.0};

    UsbDiskInfo(size_t idx, const std::string& dev, const std::string& mnt, const std::string& lbl);

    struct MetaSnapshot {
        std::string dev_name;
        std::string part_name;
        std::string fs_type;
        std::string model;
        double capacity_gb{0.0};
    };

    MetaSnapshot get_meta() const;
};

class UsbStorageTester {
public:
    std::vector<std::unique_ptr<UsbDiskInfo>> disks;
    std::atomic<bool> active{false};
    std::vector<std::thread> workers;

    UsbStorageTester();
    ~UsbStorageTester();

    void start();
    void stop();
    void reset();

    uint64_t get_total_write_mb() const;
    uint64_t get_total_read_mb() const;
    uint64_t get_total_errors() const;
    double get_total_write_speed() const;
    double get_total_read_speed() const;

private:
    std::mutex state_mtx;
    std::mutex rebind_mtx;
    std::atomic<bool> shutdown_flag{false};
    std::thread monitor_thread;

    void init_disks();

    static bool get_mount_info(const std::string& mount_point, std::string& out_dev, std::string& out_fs);
    static bool is_system_disk(const std::string& dev_name);
    static std::string get_usb_topology_path(const std::string& dev_name);
    static std::string find_disk_partition(const std::string& dev_name);
    static std::string detect_fs_type(const std::string& part);
    static void read_disk_sysfs(UsbDiskInfo& d, const std::string& dev_name);
    static void lazy_unmount(const std::string& mount_point);

    std::string find_or_rebind_dev(UsbDiskInfo& d);
    bool try_mount_disk(UsbDiskInfo& d);
    void standby_monitor_loop();
    void handle_io_failure(UsbDiskInfo& d, const std::string& test_file);
    void worker_loop(size_t idx);
};
