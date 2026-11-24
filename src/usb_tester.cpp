#include "usb_tester.hpp"
#include "common.hpp"

#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

UsbDiskInfo::UsbDiskInfo(size_t idx, const std::string& dev, const std::string& mnt, const std::string& lbl)
    : slot_idx(idx), mount_point(mnt), label(lbl), dev_name(dev) {}

UsbDiskInfo::MetaSnapshot UsbDiskInfo::get_meta() const {
    std::lock_guard<std::mutex> lk(meta_mtx);
    return {dev_name, part_name, fs_type, model, capacity_gb};
}

UsbStorageTester::UsbStorageTester() {
    init_disks();
    monitor_thread = std::thread(&UsbStorageTester::standby_monitor_loop, this);
}

UsbStorageTester::~UsbStorageTester() {
    stop();
    {
        std::lock_guard<std::mutex> lk(monitor_mtx);
        shutdown_flag.store(true);
    }
    monitor_cv.notify_all();
    if (monitor_thread.joinable()) monitor_thread.join();
}

void UsbStorageTester::start() {
    std::lock_guard<std::mutex> lk(state_mtx);
    if (active.load()) return;
    active.store(true);
    workers.clear();
    for (size_t i = 0; i < disks.size(); ++i) {
        workers.emplace_back(&UsbStorageTester::worker_loop, this, i);
    }
}

void UsbStorageTester::request_stop() {
    active.store(false);
}

void UsbStorageTester::stop() {
    std::vector<std::thread> to_join;
    {
        std::lock_guard<std::mutex> lk(state_mtx);
        if (!active.load() && workers.empty()) return;
        active.store(false);
        to_join.swap(workers);
    }
    for (auto& t : to_join) {
        if (t.joinable()) t.join();
    }
    for (auto& d : disks) {
        d->write_speed_mb.store(0.0);
        d->read_speed_mb.store(0.0);
    }
}

void UsbStorageTester::reset() {
    for (auto& d : disks) {
        d->write_bytes.store(0);
        d->read_bytes.store(0);
        d->write_ops.store(0);
        d->read_ops.store(0);
        d->errors.store(0);
        d->disconnects.store(0);
        d->recovers.store(0);
        d->write_speed_mb.store(0.0);
        d->read_speed_mb.store(0.0);
    }
}

uint64_t UsbStorageTester::get_total_write_mb() const {
    uint64_t s = 0;
    for (const auto& d : disks) s += (d->write_bytes.load() / (1024 * 1024));
    return s;
}

uint64_t UsbStorageTester::get_total_read_mb() const {
    uint64_t s = 0;
    for (const auto& d : disks) s += (d->read_bytes.load() / (1024 * 1024));
    return s;
}

uint64_t UsbStorageTester::get_total_errors() const {
    uint64_t s = 0;
    for (const auto& d : disks) s += d->errors.load();
    return s;
}

double UsbStorageTester::get_total_write_speed() const {
    double s = 0.0;
    for (const auto& d : disks) s += d->write_speed_mb.load();
    return s;
}

double UsbStorageTester::get_total_read_speed() const {
    double s = 0.0;
    for (const auto& d : disks) s += d->read_speed_mb.load();
    return s;
}

void UsbStorageTester::init_disks() {
    disks.clear();
    disks.push_back(std::make_unique<UsbDiskInfo>(0, "sda", "/mnt/rk_usb_test/sda", "USB 1"));
    disks.push_back(std::make_unique<UsbDiskInfo>(1, "sdb", "/mnt/rk_usb_test/sdb", "USB 2"));
    disks.push_back(std::make_unique<UsbDiskInfo>(2, "sdc", "/mnt/rk_usb_test/sdc", "USB 3"));
    disks.push_back(std::make_unique<UsbDiskInfo>(3, "sdd", "/mnt/rk_usb_test/sdd", "USB 4"));
    mkdir("/mnt/rk_usb_test", 0755);
    for (auto& d : disks) {
        mkdir(d->mount_point.c_str(), 0755);
    }
}

bool UsbStorageTester::get_mount_info(const std::string& mount_point, std::string& out_dev, std::string& out_fs) {
    std::ifstream f("/proc/mounts");
    if (!f.is_open()) return false;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string dev, mnt, fs;
        if (ss >> dev >> mnt >> fs) {
            if (mnt == mount_point) {
                out_dev = dev;
                out_fs = fs;
                return true;
            }
        }
    }
    return false;
}

bool UsbStorageTester::is_system_disk(const std::string& dev_name) {
    std::ifstream f("/proc/mounts");
    if (!f.is_open()) return false;
    std::string prefix = "/dev/" + dev_name;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string dev, mnt;
        if (ss >> dev >> mnt) {
            if (dev == prefix || (dev.rfind(prefix, 0) == 0 && dev.size() > prefix.size() &&
                                  dev[prefix.size()] >= '0' && dev[prefix.size()] <= '9')) {
                if (mnt.rfind("/mnt/rk_usb_test/", 0) != 0) {
                    if (mnt == "/" || mnt == "/boot" || mnt == "/boot/efi" ||
                        mnt == "/home" || mnt == "/usr" || mnt == "/var") {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

std::string UsbStorageTester::get_usb_topology_path(const std::string& dev_name) {
    char resolved[PATH_MAX];
    std::string sys_path = "/sys/block/" + dev_name;
    if (!realpath(sys_path.c_str(), resolved)) return "";
    std::string full(resolved);
    if (full.find("/usb") == std::string::npos) return "";
    size_t host_pos = full.find("/host");
    if (host_pos != std::string::npos) {
        return full.substr(0, host_pos);
    }
    return full;
}

std::string UsbStorageTester::find_disk_partition(const std::string& dev_name) {
    if (!is_valid_block_dev_name(dev_name)) return "";
    if (access(("/sys/block/" + dev_name).c_str(), F_OK) != 0) return "";
    if (is_system_disk(dev_name)) return "";

    std::string best_part;
    long long max_size = 0;
    // 遍历分区 1..8，挑选容量最大的数据分区（避开 16MB EFI / MSR 分区）
    for (int p = 1; p <= 8; ++p) {
        std::string part_name = dev_name + std::to_string(p);
        std::string size_file = "/sys/block/" + dev_name + "/" + part_name + "/size";
        std::ifstream sf(size_file);
        if (sf.is_open()) {
            long long sz = 0;
            if (sf >> sz && sz > max_size) {
                max_size = sz;
                best_part = "/dev/" + part_name;
            }
        }
    }
    if (!best_part.empty() && access(best_part.c_str(), F_OK) == 0) return best_part;

    // 回退：无分区表的裸盘（如整盘格式化 U 盘）
    std::string raw_dev = "/dev/" + dev_name;
    if (access(raw_dev.c_str(), F_OK) == 0) return raw_dev;
    return "";
}

// 热插拔盘符漂移恢复（如 sda 拔插后内核重分配为 sde）
std::string UsbStorageTester::find_or_rebind_dev(UsbDiskInfo& d) {
    std::lock_guard<std::mutex> rlk(rebind_mtx);
    std::string cur_dev;
    std::string saved_usb_path;
    {
        std::lock_guard<std::mutex> mlk(d.meta_mtx);
        cur_dev = d.dev_name;
        saved_usb_path = d.usb_port_path;
    }

    if (!find_disk_partition(cur_dev).empty()) {
        std::string topo = get_usb_topology_path(cur_dev);
        if (!topo.empty() && saved_usb_path.empty()) {
            std::lock_guard<std::mutex> mlk(d.meta_mtx);
            d.usb_port_path = topo;
        }
        return cur_dev;
    }

    // 收集当前已被其他槽位占用的块设备名
    std::set<std::string> claimed;
    for (const auto& other : disks) {
        if (other.get() == &d) continue;
        auto m = other->get_meta();
        if (!m.dev_name.empty()) claimed.insert(m.dev_name);
    }

    DIR* dir = opendir("/sys/block");
    if (!dir) return "";

    std::string exact_topo_match;
    std::string fallback_usb_dev;
    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr) {
        std::string cand = ent->d_name;
        if (!is_valid_block_dev_name(cand)) continue;
        if (claimed.count(cand)) continue;
        std::string topo = get_usb_topology_path(cand);
        if (topo.empty()) continue; // 仅允许自动绑定确认属于 USB 总线的块设备
        if (find_disk_partition(cand).empty()) continue;

        if (!saved_usb_path.empty() && topo == saved_usb_path) {
            exact_topo_match = cand;
            break;
        }
        if (fallback_usb_dev.empty()) {
            fallback_usb_dev = cand;
        }
    }
    closedir(dir);

    std::string chosen = !exact_topo_match.empty() ? exact_topo_match : fallback_usb_dev;
    if (!chosen.empty()) {
        std::string topo = get_usb_topology_path(chosen);
        std::lock_guard<std::mutex> mlk(d.meta_mtx);
        d.dev_name = chosen;
        if (!topo.empty()) d.usb_port_path = topo;
    }
    return chosen;
}

std::string UsbStorageTester::detect_fs_type(const std::string& part) {
    if (!is_valid_part_path(part)) return "";
    std::string cmd = "blkid -o value -s TYPE " + part + " 2>/dev/null";
    FILE* fp = popen(cmd.c_str(), "r");
    if (!fp) return "";
    char buf[64]{};
    std::string fs;
    if (fgets(buf, sizeof(buf), fp)) {
        fs = buf;
        while (!fs.empty() && (fs.back() == '\n' || fs.back() == '\r' || fs.back() == ' ')) {
            fs.pop_back();
        }
    }
    pclose(fp);
    return fs;
}

void UsbStorageTester::read_disk_sysfs(UsbDiskInfo& d, const std::string& dev_name) {
    std::string mod;
    std::ifstream fm("/sys/block/" + dev_name + "/device/model");
    if (fm.is_open()) {
        std::getline(fm, mod);
        while (!mod.empty() && (mod.back() == ' ' || mod.back() == '\r' || mod.back() == '\n')) {
            mod.pop_back();
        }
    }
    double cap_gb = 0.0;
    std::ifstream fs("/sys/block/" + dev_name + "/size");
    long long sec = 0;
    if (fs.is_open() && (fs >> sec)) {
        cap_gb = static_cast<double>(sec) * 512.0 / (1024.0 * 1024.0 * 1024.0);
    }
    std::lock_guard<std::mutex> lk(d.meta_mtx);
    if (!mod.empty()) d.model = mod;
    if (cap_gb > 0.0) d.capacity_gb = cap_gb;
}

void UsbStorageTester::lazy_unmount(const std::string& mount_point) {
    std::string umnt = "umount -l " + mount_point + " 2>/dev/null";
    if (system(umnt.c_str()) < 0) {}
}

bool UsbStorageTester::try_mount_disk(UsbDiskInfo& d) {
    std::lock_guard<std::mutex> mlk(d.mount_mtx);
    std::string dev = find_or_rebind_dev(d);
    if (dev.empty()) {
        std::string mnt_dev, mnt_fs;
        if (d.mounted.load() || get_mount_info(d.mount_point, mnt_dev, mnt_fs)) {
            d.mounted.store(false);
            lazy_unmount(d.mount_point);
        }
        return false;
    }

    std::string part = find_disk_partition(dev);
    if (part.empty()) {
        if (d.mounted.load()) {
            d.mounted.store(false);
            lazy_unmount(d.mount_point);
        }
        return false;
    }

    // 检查现有挂载是否依然指向有效设备
    std::string mnt_dev, mnt_fs;
    if (get_mount_info(d.mount_point, mnt_dev, mnt_fs)) {
        if (mnt_dev == part && access(part.c_str(), F_OK) == 0) {
            {
                std::lock_guard<std::mutex> lk(d.meta_mtx);
                d.part_name = part;
                if (!mnt_fs.empty()) d.fs_type = mnt_fs;
            }
            read_disk_sysfs(d, dev);
            d.mounted.store(true);
            return true;
        }
        // 旧的陈旧挂载节点（比如盘符漂移前残留的 FUSE 挂载），先卸载
        lazy_unmount(d.mount_point);
    }

    if (mkdir(d.mount_point.c_str(), 0755) < 0 && errno != EEXIST) {}

    std::string detected_fs = detect_fs_type(part);
    std::string mnt_cmd;
    if (detected_fs == "exfat") {
        mnt_cmd = "mount -t exfat " + part + " " + d.mount_point + " 2>/dev/null || "
                  "mount.exfat-fuse " + part + " " + d.mount_point + " 2>/dev/null || "
                  "mount " + part + " " + d.mount_point + " 2>/dev/null";
    } else if (detected_fs == "ntfs") {
        mnt_cmd = "mount -t ntfs-3g " + part + " " + d.mount_point + " 2>/dev/null || "
                  "mount -t ntfs3 " + part + " " + d.mount_point + " 2>/dev/null || "
                  "mount " + part + " " + d.mount_point + " 2>/dev/null";
    } else {
        mnt_cmd = "mount " + part + " " + d.mount_point + " 2>/dev/null || "
                  "mount -t ntfs-3g " + part + " " + d.mount_point + " 2>/dev/null || "
                  "mount.exfat-fuse " + part + " " + d.mount_point + " 2>/dev/null";
    }

    if (system(mnt_cmd.c_str()) == 0) {
        std::string actual_dev, actual_fs;
        get_mount_info(d.mount_point, actual_dev, actual_fs);
        {
            std::lock_guard<std::mutex> lk(d.meta_mtx);
            d.part_name = part;
            d.fs_type = !detected_fs.empty() ? detected_fs : (!actual_fs.empty() ? actual_fs : "auto");
        }
        read_disk_sysfs(d, dev);
        d.mounted.store(true);
        return true;
    }

    d.mounted.store(false);
    return false;
}

// 待机模式后台热插拔巡检线程（不持有 state_mtx，绝不阻塞主线程启动/停止 USB 压测）
void UsbStorageTester::standby_monitor_loop() {
    while (!shutdown_flag.load()) {
        if (!active.load()) {
            for (auto& d : disks) {
                if (shutdown_flag.load() || active.load()) break;
                try_mount_disk(*d);
            }
        }
        std::unique_lock<std::mutex> lk(monitor_mtx);
        monitor_cv.wait_for(lk, std::chrono::milliseconds(1000),
                            [this]() { return shutdown_flag.load(); });
    }
}

void UsbStorageTester::handle_io_failure(UsbDiskInfo& d, const std::string& test_file) {
    if (active.load()) {
        d.errors.fetch_add(1);
        d.write_speed_mb.store(0.0);
        d.read_speed_mb.store(0.0);
        auto meta = d.get_meta();
        if (find_disk_partition(meta.dev_name).empty()) {
            if (d.mounted.exchange(false)) {
                d.disconnects.fetch_add(1);
            }
            lazy_unmount(d.mount_point);
        }
    }
    unlink(test_file.c_str());
}

void UsbStorageTester::worker_loop(size_t idx) {
    auto& d = *disks[idx];
    // 4MB 4K对齐数据块 + 持久化文件 + O_DIRECT（不支持 O_DIRECT 的 FUSE 文件系统自动回退 fdatasync+fadvise），
    // 去除 20ms 人工限速与每轮 open/unlink 元数据开销，充分压榨 USB 3.0 真实读写带宽
    constexpr size_t BLK_SIZE = 4 * 1024 * 1024;
    void* raw_w = nullptr;
    void* raw_r = nullptr;
    if (posix_memalign(&raw_w, 4096, BLK_SIZE) != 0 || posix_memalign(&raw_r, 4096, BLK_SIZE) != 0 ||
        !raw_w || !raw_r) {
        if (raw_w) free(raw_w);
        if (raw_r) free(raw_r);
        return;
    }
    auto* w_buf = static_cast<uint8_t*>(raw_w);
    auto* r_buf = static_cast<uint8_t*>(raw_r);
    auto* w_u32 = reinterpret_cast<uint32_t*>(raw_w);
    constexpr size_t kWords = BLK_SIZE / sizeof(uint32_t);

    uint32_t seed = static_cast<uint32_t>(idx * 1000003u + 1u);
    auto last_calc = std::chrono::steady_clock::now();
    uint64_t w_since_calc = 0;
    uint64_t r_since_calc = 0;
    bool was_disconnected = false;

    std::string test_file = d.mount_point + "/.rk_test_io_slot" + std::to_string(idx) + ".dat";
    int io_fd = -1;
    bool use_direct = false;

    auto close_io_fd = [&]() {
        if (io_fd >= 0) {
            close(io_fd);
            io_fd = -1;
        }
        use_direct = false;
    };

    while (active.load()) {
        if (!d.mounted.load()) {
            close_io_fd();
            d.write_speed_mb.store(0.0);
            d.read_speed_mb.store(0.0);

            if (try_mount_disk(d)) {
                if (was_disconnected || d.disconnects.load() > 0) {
                    d.recovers.fetch_add(1);
                }
                was_disconnected = false;
                last_calc = std::chrono::steady_clock::now();
                w_since_calc = 0;
                r_since_calc = 0;
            } else {
                was_disconnected = true;
                for (int s = 0; s < 25 && active.load(); ++s) usleep(20000);
                continue;
            }
        }

        if (io_fd < 0) {
            // 优先 O_DIRECT 零页缓存高速直写/直读；FUSE(ntfs-3g/exfat-fuse) 等不支持时无缝回退
            io_fd = open(test_file.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_DIRECT, 0666);
            if (io_fd >= 0) {
                use_direct = true;
            } else {
                io_fd = open(test_file.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
                use_direct = false;
            }
            if (io_fd < 0) {
                handle_io_failure(d, test_file);
                w_since_calc = 0;
                r_since_calc = 0;
                for (int s = 0; s < 15 && active.load(); ++s) usleep(20000);
                continue;
            }
        }

        for (size_t i = 0; i < kWords; ++i) {
            w_u32[i] = seed + static_cast<uint32_t>(i * 2654435761u);
        }
        seed += 1013904223u;

        // 1. Write phase (pwrite offset 0，每次 1MB 切片兼顾 4MB 连续吞吐与快速响应退出)
        constexpr size_t kIoSlice = 1024 * 1024;
        size_t written = 0;
        bool write_err = false;
        while (written < BLK_SIZE && active.load()) {
            size_t chunk = std::min(kIoSlice, BLK_SIZE - written);
            ssize_t w = pwrite(io_fd, w_buf + written, chunk, static_cast<off_t>(written));
            if (w <= 0) {
                if (w < 0 && errno == EINTR) continue;
                // 若文件系统中途拒绝 O_DIRECT (EINVAL)，切回缓冲模式重试一次
                if (w < 0 && errno == EINVAL && use_direct && written == 0) {
                    close_io_fd();
                    io_fd = open(test_file.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
                    if (io_fd >= 0) continue;
                }
                write_err = true;
                break;
            }
            written += static_cast<size_t>(w);
        }
        if (written == BLK_SIZE && !write_err && !use_direct && active.load()) {
            if (fdatasync(io_fd) < 0) {
                write_err = true;
            } else {
                posix_fadvise(io_fd, 0, static_cast<off_t>(BLK_SIZE), POSIX_FADV_DONTNEED);
            }
        }

        if (!active.load()) break;
        if (write_err || written < BLK_SIZE) {
            close_io_fd();
            handle_io_failure(d, test_file);
            w_since_calc = 0;
            r_since_calc = 0;
            for (int s = 0; s < 15 && active.load(); ++s) usleep(20000);
            continue;
        }

        d.write_bytes.fetch_add(written);
        d.write_ops.fetch_add(1);
        w_since_calc += written;

        // 2. Read phase (pread offset 0，绕过页缓存真实从 USB 介质回读)
        if (!use_direct) {
            posix_fadvise(io_fd, 0, static_cast<off_t>(BLK_SIZE), POSIX_FADV_DONTNEED);
        }
        size_t read_bytes = 0;
        bool read_err = false;
        while (read_bytes < BLK_SIZE && active.load()) {
            size_t chunk = std::min(kIoSlice, BLK_SIZE - read_bytes);
            ssize_t r = pread(io_fd, r_buf + read_bytes, chunk, static_cast<off_t>(read_bytes));
            if (r <= 0) {
                if (r < 0 && errno == EINTR) continue;
                read_err = true;
                break;
            }
            read_bytes += static_cast<size_t>(r);
        }
        if (!use_direct && io_fd >= 0) {
            posix_fadvise(io_fd, 0, static_cast<off_t>(BLK_SIZE), POSIX_FADV_DONTNEED);
        }

        if (!active.load()) break;
        if (read_err || read_bytes < BLK_SIZE) {
            close_io_fd();
            handle_io_failure(d, test_file);
            w_since_calc = 0;
            r_since_calc = 0;
            for (int s = 0; s < 15 && active.load(); ++s) usleep(20000);
            continue;
        }

        d.read_bytes.fetch_add(read_bytes);
        d.read_ops.fetch_add(1);
        r_since_calc += read_bytes;

        // 3. Verify 4MB data integrity
        if (std::memcmp(w_buf, r_buf, BLK_SIZE) != 0) {
            if (active.load()) d.errors.fetch_add(1);
        }

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calc).count() / 1000.0;
        if (dt >= 0.5) {
            last_calc = now;
            double w_mb_s = (w_since_calc / (1024.0 * 1024.0)) / dt;
            double r_mb_s = (r_since_calc / (1024.0 * 1024.0)) / dt;
            d.write_speed_mb.store(w_mb_s);
            d.read_speed_mb.store(r_mb_s);
            w_since_calc = 0;
            r_since_calc = 0;
        }
    }

    close_io_fd();
    unlink(test_file.c_str());
    free(raw_w);
    free(raw_r);
    d.write_speed_mb.store(0.0);
    d.read_speed_mb.store(0.0);
}
