#include "accel_burn.hpp"
#include "common.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

// Linux DMA-BUF Heap ioctl ABI (<linux/dma-heap.h>)
struct DmaHeapAllocationData {
    uint64_t len;
    uint32_t fd;
    uint32_t fd_flags;
    uint64_t heap_flags;
};

constexpr unsigned long kDmaHeapIoctlAlloc = _IOWR('H', 0x0, DmaHeapAllocationData);

int alloc_dma_heap_buf(size_t size) {
    int heap_fd = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (heap_fd < 0) return -1;
    DmaHeapAllocationData req{};
    req.len = size;
    req.fd_flags = O_RDWR | O_CLOEXEC;
    req.heap_flags = 0;
    int ret = ioctl(heap_fd, kDmaHeapIoctlAlloc, &req);
    close(heap_fd);
    if (ret < 0) return -1;
    return static_cast<int>(req.fd);
}

// ---------------------------------------------------------------------------
// Rockchip RGA IM2D ABI 镜像定义（与 librga v1.10.6 精确字节对齐）
// ---------------------------------------------------------------------------
using RgaBufHandle = uint32_t;

struct RgaRect {
    int x;
    int y;
    int width;
    int height;
};

struct RgaColorKeyRange {
    int max;
    int min;
};

struct RgaNn {
    int scale_r, scale_g, scale_b;
    int offset_r, offset_g, offset_b;
};

struct RgaBuffer {
    void* vir_addr;
    void* phy_addr;
    int fd;
    int width;
    int height;
    int wstride;
    int hstride;
    int format;
    int color_space_mode;
    int global_alpha;
    int rd_mode;
    int color;
    RgaColorKeyRange colorkey_range;
    RgaNn nn;
    int rop_code;
    RgaBufHandle handle;
};
static_assert(sizeof(RgaBuffer) == 96, "RgaBuffer ABI mismatch with librga");

struct RgaHandleParam {
    uint32_t width;
    uint32_t height;
    uint32_t format;
};
static_assert(sizeof(RgaHandleParam) == 12, "RgaHandleParam ABI mismatch");

struct RgaOpt {
    uint32_t version;
    int color;
    RgaColorKeyRange colorkey_range;
    RgaNn nn;
    int rop_code;
    int priority;
    int core;
    uint8_t tail[312 - 52];
};
static_assert(sizeof(RgaOpt) == 312, "RgaOpt ABI mismatch with librga");
static_assert(offsetof(RgaOpt, core) == 48, "RgaOpt::core offset mismatch");

constexpr uint32_t kRgaApiVersion = (1u << 24) | (10u << 16) | (6u << 8) | 3u;
constexpr int kRgaFmtRgba8888 = 0x0 << 8;
constexpr int kRgaFmtNv12 = 0xa << 8;
constexpr int kRgaSchedRga2Core0 = 1 << 2;
constexpr int kRgaSchedRga2Core1 = 1 << 3;
constexpr int kRgaUsageRot180 = 1 << 1;
constexpr int kRgaUsageSync = 1 << 19;
constexpr int kRgaStatusSuccess = 1;

// ---------------------------------------------------------------------------
// Rockchip MPP ABI 镜像定义（与 librockchip_mpp 精确对齐）
// ---------------------------------------------------------------------------
using MppCtx = void*;
using MppParam = void*;
using MppFrame = void*;
using MppPacket = void*;
using MppBuffer = void*;
using MppBufferGroup = void*;
using MppTask = void*;
using MppEncCfg = void*;
using MppDecCfg = void*;
using MppRet = int32_t;

struct MppApiTable {
    uint32_t size;
    uint32_t version;
    MppRet (*decode)(MppCtx, MppPacket, MppFrame*);
    MppRet (*decode_put_packet)(MppCtx, MppPacket);
    MppRet (*decode_get_frame)(MppCtx, MppFrame*);
    MppRet (*encode)(MppCtx, MppFrame, MppPacket*);
    MppRet (*encode_put_frame)(MppCtx, MppFrame);
    MppRet (*encode_get_packet)(MppCtx, MppPacket*);
    MppRet (*isp)(MppCtx, MppFrame, MppFrame);
    MppRet (*isp_put_frame)(MppCtx, MppFrame);
    MppRet (*isp_get_frame)(MppCtx, MppFrame*);
    MppRet (*poll)(MppCtx, int, int);
    MppRet (*dequeue)(MppCtx, int, MppTask*);
    MppRet (*enqueue)(MppCtx, int, MppTask);
    MppRet (*reset)(MppCtx);
    MppRet (*control)(MppCtx, int, MppParam);
    uint32_t reserv[16];
};
static_assert(sizeof(MppApiTable) == 184, "MppApiTable ABI mismatch");
static_assert(offsetof(MppApiTable, decode_put_packet) == 16, "MppApiTable offset mismatch");
static_assert(offsetof(MppApiTable, encode_put_frame) == 40, "MppApiTable offset mismatch");
static_assert(offsetof(MppApiTable, control) == 112, "MppApiTable offset mismatch");

constexpr int kMppCtxDec = 0;
constexpr int kMppCtxEnc = 1;
constexpr int kMppCodingAvc = 7;
constexpr int kMppCodingHevc = 0x01000004;
constexpr int kMppFmtYuv420Sp = 0;
constexpr int kMppBufTypeDrm = 3;
constexpr int kMppBufInternal = 0;
constexpr int kMppRcModeCbr = 1;
constexpr int kMppCmdSetInputTimeout = 0x200006;
constexpr int kMppCmdSetOutputTimeout = 0x200007;
constexpr int kMppCmdDecSetExtBufGroup = 0x310002;
constexpr int kMppCmdDecSetInfoChangeReady = 0x310003;
constexpr int kMppCmdDecSetCfg = 0x310201;
constexpr int kMppCmdDecGetCfg = 0x310202;
constexpr int kMppCmdEncSetCfg = 0x320001;
constexpr int kMppCmdEncGetCfg = 0x320002;
constexpr int kMppCmdEncGetHdrSync = 0x32000d;
constexpr int kMppErrTimeout = -8;

void fill_nv12_pattern(uint8_t* ptr, int w, int h, int phase) {
    if (!ptr) return;
    // 生成高复杂度棋盘+渐变+动态偏移纹理，最大化 H.265 运动估计与 CABAC 硬件计算负载
    for (int y = 0; y < h; ++y) {
        uint8_t* row = ptr + static_cast<size_t>(y) * w;
        int by = ((y + phase * 37) >> 4);
        for (int x = 0; x < w; x += 16) {
            int bx = ((x + phase * 53) >> 4);
            uint8_t base = static_cast<uint8_t>(((bx ^ by) * 29 + phase * 61 + (y & 15) * 9) & 0xFF);
            for (int k = 0; k < 16 && (x + k) < w; ++k) {
                row[x + k] = static_cast<uint8_t>(base ^ (k * 13 + (y & 7) * 7));
            }
        }
    }
    uint8_t* uv = ptr + static_cast<size_t>(w) * h;
    int uv_h = h / 2;
    for (int y = 0; y < uv_h; ++y) {
        uint8_t* row = uv + static_cast<size_t>(y) * w;
        uint8_t uval = static_cast<uint8_t>((64 + ((y + phase * 23) & 127)) & 0xFF);
        uint8_t vval = static_cast<uint8_t>((192 - ((y + phase * 31) & 127)) & 0xFF);
        for (int x = 0; x < w; x += 2) {
            row[x] = static_cast<uint8_t>(uval ^ ((x >> 4) & 31));
            row[x + 1] = static_cast<uint8_t>(vval ^ ((x >> 3) & 31));
        }
    }
}

} // namespace

int run_internal_gpu_opencl_burn() {
    using cl_int = int32_t;
    using cl_uint = uint32_t;
    using cl_ulong = uint64_t;
    using cl_platform_id = void*;
    using cl_device_id = void*;
    using cl_context = void*;
    using cl_command_queue = void*;
    using cl_program = void*;
    using cl_kernel = void*;
    using cl_mem = void*;
    constexpr cl_ulong kDevGpu = (1ULL << 2);
    constexpr cl_ulong kMemRW = (1ULL << 0);

    void* lib = dlopen("libMaliOpenCL.so.1", RTLD_NOW);
    if (!lib) lib = dlopen("/usr/lib/aarch64-linux-gnu/mali/libMaliOpenCL.so.1", RTLD_NOW);
    if (!lib) lib = dlopen("libOpenCL.so.1", RTLD_NOW);
    if (!lib) return 1;

    auto pGetPlat = reinterpret_cast<cl_int(*)(cl_uint, cl_platform_id*, cl_uint*)>(dlsym(lib, "clGetPlatformIDs"));
    auto pGetDev = reinterpret_cast<cl_int(*)(cl_platform_id, cl_ulong, cl_uint, cl_device_id*, cl_uint*)>(dlsym(lib, "clGetDeviceIDs"));
    auto pCreateCtx = reinterpret_cast<cl_context(*)(const void*, cl_uint, const cl_device_id*, void*, void*, cl_int*)>(dlsym(lib, "clCreateContext"));
    auto pCreateQ = reinterpret_cast<cl_command_queue(*)(cl_context, cl_device_id, cl_ulong, cl_int*)>(dlsym(lib, "clCreateCommandQueue"));
    auto pCreateBuf = reinterpret_cast<cl_mem(*)(cl_context, cl_ulong, size_t, void*, cl_int*)>(dlsym(lib, "clCreateBuffer"));
    auto pCreateProg = reinterpret_cast<cl_program(*)(cl_context, cl_uint, const char**, const size_t*, cl_int*)>(dlsym(lib, "clCreateProgramWithSource"));
    auto pBuildProg = reinterpret_cast<cl_int(*)(cl_program, cl_uint, const cl_device_id*, const char*, void*, void*)>(dlsym(lib, "clBuildProgram"));
    auto pCreateKern = reinterpret_cast<cl_kernel(*)(cl_program, const char*, cl_int*)>(dlsym(lib, "clCreateKernel"));
    auto pSetArg = reinterpret_cast<cl_int(*)(cl_kernel, cl_uint, size_t, const void*)>(dlsym(lib, "clSetKernelArg"));
    auto pEnqND = reinterpret_cast<cl_int(*)(cl_command_queue, cl_kernel, cl_uint, const size_t*, const size_t*, const size_t*, cl_uint, const void*, void*)>(dlsym(lib, "clEnqueueNDRangeKernel"));
    auto pFinish = reinterpret_cast<cl_int(*)(cl_command_queue)>(dlsym(lib, "clFinish"));
    if (!pGetPlat || !pGetDev || !pCreateCtx || !pCreateQ || !pCreateBuf ||
        !pCreateProg || !pBuildProg || !pCreateKern || !pSetArg || !pEnqND || !pFinish) {
        return 2;
    }

    cl_platform_id plat = nullptr;
    cl_uint nplat = 0;
    if (pGetPlat(1, &plat, &nplat) != 0 || nplat == 0) return 3;

    cl_device_id dev = nullptr;
    cl_uint ndev = 0;
    if (pGetDev(plat, kDevGpu, 1, &dev, &ndev) != 0 || ndev == 0) return 4;

    cl_int err = 0;
    cl_context ctx = pCreateCtx(nullptr, 1, &dev, nullptr, nullptr, &err);
    if (!ctx || err != 0) return 5;
    cl_command_queue q = pCreateQ(ctx, dev, 0, &err);
    if (!q || err != 0) return 6;

    const size_t N = 262144;
    cl_mem buf = pCreateBuf(ctx, kMemRW, N * sizeof(float), nullptr, &err);
    if (!buf || err != 0) return 7;

    const char* src =
        "__kernel void gpu_burn(__global float* out) {"
        "  int id = get_global_id(0);"
        "  float4 a = (float4)((float)(id & 255) * 0.001f + 1.0f, 1.1f, 1.2f, 1.3f);"
        "  float4 b = (float4)(1.0001f, 0.9999f, 1.0002f, 0.9998f);"
        "  float4 c = (float4)(0.0001f, 0.0002f, 0.0003f, 0.0004f);"
        "  for (int i = 0; i < 4096; ++i) {"
        "    a = fma(a, b, c);"
        "    if (a.x > 1000.0f) a *= 0.001f;"
        "  }"
        "  out[id] = a.x + a.y + a.z + a.w;"
        "}";
    cl_program prog = pCreateProg(ctx, 1, &src, nullptr, &err);
    if (!prog || err != 0) return 8;
    if (pBuildProg(prog, 1, &dev, "", nullptr, nullptr) != 0) return 9;
    cl_kernel kern = pCreateKern(prog, "gpu_burn", &err);
    if (!kern || err != 0) return 10;
    if (pSetArg(kern, 0, sizeof(cl_mem), &buf) != 0) return 11;

    while (!g_quit_flag) {
        for (int r = 0; r < 4; ++r) {
            if (pEnqND(q, kern, 1, nullptr, &N, nullptr, 0, nullptr, nullptr) != 0) return 12;
        }
        if (pFinish(q) != 0) return 13;
    }
    return 0;
}

int run_internal_npu_matmul_burn() {
    using RknnCtx = uint64_t;
    struct RknnTensorMem {
        void* virt_addr;
        uint64_t phys_addr;
        int32_t fd;
        int32_t offset;
        uint32_t size;
        uint32_t flags;
        void* priv_data;
    };
    struct RknnMatmulTensorAttr {
        char name[256];
        uint32_t n_dims;
        uint32_t dims[16];
        uint32_t size;
        int32_t type;
    };
    struct RknnMatmulIoAttr {
        RknnMatmulTensorAttr A;
        RknnMatmulTensorAttr B;
        RknnMatmulTensorAttr C;
    };
    struct RknnMatmulInfo {
        int32_t M;
        int32_t K;
        int32_t N;
        int32_t type;
        int16_t B_layout;
        int16_t B_quant_type;
        int16_t AC_layout;
        int16_t AC_quant_type;
        int32_t iommu_domain_id;
        int16_t group_size;
        int8_t reserved[34];
    };

    void* lib = dlopen("librknnrt.so", RTLD_NOW);
    if (!lib) lib = dlopen("/usr/lib/aarch64-linux-gnu/librknnrt.so", RTLD_NOW);
    if (!lib) return 1;

    auto pCreate = reinterpret_cast<int(*)(RknnCtx*, RknnMatmulInfo*, RknnMatmulIoAttr*)>(dlsym(lib, "rknn_matmul_create"));
    auto pSetCore = reinterpret_cast<int(*)(RknnCtx, int)>(dlsym(lib, "rknn_matmul_set_core_mask"));
    auto pCreateMem = reinterpret_cast<RknnTensorMem*(*)(RknnCtx, uint32_t)>(dlsym(lib, "rknn_create_mem"));
    auto pSetIoMem = reinterpret_cast<int(*)(RknnCtx, RknnTensorMem*, RknnMatmulTensorAttr*)>(dlsym(lib, "rknn_matmul_set_io_mem"));
    auto pRun = reinterpret_cast<int(*)(RknnCtx)>(dlsym(lib, "rknn_matmul_run"));
    auto pDestroyMem = reinterpret_cast<int(*)(RknnCtx, RknnTensorMem*)>(dlsym(lib, "rknn_destroy_mem"));
    auto pDestroy = reinterpret_cast<int(*)(RknnCtx)>(dlsym(lib, "rknn_matmul_destroy"));
    if (!pCreate || !pCreateMem || !pSetIoMem || !pRun || !pDestroyMem || !pDestroy) return 2;

    std::atomic<bool> failed{false};
    auto npu_worker = [&](int core_mask) {
        // 优先使用超大尺寸 FP16 矩阵乘以拉长单次硬件计算时间，确保 950MHz 满频下双核 NPU 仍保持 95%~100%
        struct ShapeCfg { int32_t M, K, N, type; };
        const ShapeCfg cfgs[] = {
            {2048, 2048, 1024, 1}, // RKNN_FLOAT16_MM_FLOAT16_TO_FLOAT32 (满频防 ioctl 瓶颈)
            {1024, 2048, 1024, 1}, // RKNN_FLOAT16_MM_FLOAT16_TO_FLOAT32
            {1024, 1024, 1024, 2}, // RKNN_INT8_MM_INT8_TO_INT32
            {512,  512,  512,  2}
        };
        RknnCtx ctx = 0;
        RknnMatmulIoAttr io{};
        bool created = false;
        for (const auto& sc : cfgs) {
            RknnMatmulInfo info{};
            info.M = sc.M;
            info.K = sc.K;
            info.N = sc.N;
            info.type = sc.type;
            if (pCreate(&ctx, &info, &io) == 0) {
                created = true;
                break;
            }
        }
        if (!created) {
            failed.store(true);
            return;
        }
        if (pSetCore) pSetCore(ctx, core_mask);

        RknnTensorMem* A = pCreateMem(ctx, io.A.size);
        RknnTensorMem* B = pCreateMem(ctx, io.B.size);
        RknnTensorMem* C = pCreateMem(ctx, io.C.size);
        if (!A || !B || !C) {
            if (A) pDestroyMem(ctx, A);
            if (B) pDestroyMem(ctx, B);
            if (C) pDestroyMem(ctx, C);
            pDestroy(ctx);
            failed.store(true);
            return;
        }
        if (A->virt_addr) std::memset(A->virt_addr, 0x3C, io.A.size);
        if (B->virt_addr) std::memset(B->virt_addr, 0x3C, io.B.size);
        pSetIoMem(ctx, A, &io.A);
        pSetIoMem(ctx, B, &io.B);
        pSetIoMem(ctx, C, &io.C);

        while (!g_quit_flag) {
            if (pRun(ctx) != 0) {
                failed.store(true);
                break;
            }
        }

        pDestroyMem(ctx, A);
        pDestroyMem(ctx, B);
        pDestroyMem(ctx, C);
        pDestroy(ctx);
    };

    // RK3576 双核 NPU：Core 0 (mask=1) 与 Core 1 (mask=2) 并行满载
    std::thread t0(npu_worker, 1);
    std::thread t1(npu_worker, 2);
    t0.join();
    t1.join();
    return failed.load() ? 3 : 0;
}

int run_internal_rga_burn() {
    void* lib = dlopen("librga.so", RTLD_NOW);
    if (!lib) lib = dlopen("/usr/lib/aarch64-linux-gnu/librga.so", RTLD_NOW);
    if (!lib) lib = dlopen("librga.so.2", RTLD_NOW);
    if (!lib) return 1;

    auto pImportFd = reinterpret_cast<RgaBufHandle(*)(int, RgaHandleParam*)>(dlsym(lib, "importbuffer_fd"));
    auto pImportVa = reinterpret_cast<RgaBufHandle(*)(void*, RgaHandleParam*)>(dlsym(lib, "importbuffer_virtualaddr"));
    auto pRelease = reinterpret_cast<int(*)(RgaBufHandle)>(dlsym(lib, "releasebuffer_handle"));
    auto pWrapHandle = reinterpret_cast<RgaBuffer(*)(RgaBufHandle, int, int, int, int, int)>(dlsym(lib, "wrapbuffer_handle_t"));
    auto pProcessOpt = reinterpret_cast<int(*)(RgaBuffer, RgaBuffer, RgaBuffer, RgaRect, RgaRect, RgaRect, int, int*, RgaOpt*, int)>(dlsym(lib, "improcessOpt"));
    if (!pImportFd || !pImportVa || !pRelease || !pWrapHandle || !pProcessOpt) return 2;

    // 4K (3840x2160) RGBA8888 <-> NV12 旋转+色彩空间转换，同时压榨双核 RGA2 与 DDR 带宽
    constexpr int kW = 3840;
    constexpr int kH = 2160;
    constexpr size_t kRgbaSize = static_cast<size_t>(kW) * kH * 4;
    constexpr size_t kNv12Size = static_cast<size_t>(kW) * kH * 3 / 2;

    struct RgaSurf {
        int dma_fd{-1};
        void* va{nullptr};
        size_t size{0};
        RgaBufHandle handle{0};
        RgaBuffer buf{};
    };

    auto alloc_surf = [&](int w, int h, int fmt, size_t sz, uint8_t init_val) -> RgaSurf {
        RgaSurf s;
        s.size = sz;
        RgaHandleParam hp{static_cast<uint32_t>(w), static_cast<uint32_t>(h), static_cast<uint32_t>(fmt)};
        s.dma_fd = alloc_dma_heap_buf(sz);
        if (s.dma_fd >= 0) {
            void* mapped = mmap(nullptr, sz, PROT_READ | PROT_WRITE, MAP_SHARED, s.dma_fd, 0);
            if (mapped != MAP_FAILED) {
                std::memset(mapped, init_val, sz);
                munmap(mapped, sz);
            }
            s.handle = pImportFd(s.dma_fd, &hp);
        }
        if (s.handle == 0) {
            if (s.dma_fd >= 0) {
                close(s.dma_fd);
                s.dma_fd = -1;
            }
            if (posix_memalign(&s.va, 4096, sz) == 0 && s.va) {
                std::memset(s.va, init_val, sz);
                s.handle = pImportVa(s.va, &hp);
            }
        }
        if (s.handle != 0) {
            s.buf = pWrapHandle(s.handle, w, h, w, h, fmt);
        }
        return s;
    };

    auto free_surf = [&](RgaSurf& s) {
        if (s.handle != 0) {
            pRelease(s.handle);
            s.handle = 0;
        }
        if (s.dma_fd >= 0) {
            close(s.dma_fd);
            s.dma_fd = -1;
        }
        if (s.va) {
            free(s.va);
            s.va = nullptr;
        }
    };

    // 每个 RGA2 核心分配 2 个并发工作线程，消除 ioctl 提交间隔，稳定拉满双核 RGA2 至 95%~100%
    std::atomic<bool> failed{false};
    auto rga_worker = [&](int core_mask, uint8_t seed) {
        RgaSurf rgba_a = alloc_surf(kW, kH, kRgaFmtRgba8888, kRgbaSize, static_cast<uint8_t>(0x35 + seed));
        RgaSurf nv12_b = alloc_surf(kW, kH, kRgaFmtNv12, kNv12Size, static_cast<uint8_t>(0x6A + seed));
        if (rgba_a.handle == 0 || nv12_b.handle == 0) {
            free_surf(rgba_a);
            free_surf(nv12_b);
            failed.store(true);
            return;
        }

        RgaBuffer empty_pat{};
        RgaRect full_rect{0, 0, kW, kH};
        RgaRect empty_rect{0, 0, 0, 0};
        RgaOpt opt{};
        opt.version = kRgaApiVersion;
        opt.core = core_mask;

        while (!g_quit_flag) {
            // RGBA8888 -> NV12 (CSC + 180度旋转)
            int r1 = pProcessOpt(rgba_a.buf, nv12_b.buf, empty_pat,
                                 full_rect, full_rect, empty_rect,
                                 -1, nullptr, &opt, kRgaUsageRot180 | kRgaUsageSync);
            if (r1 != kRgaStatusSuccess) {
                failed.store(true);
                break;
            }
            if (g_quit_flag) break;
            // NV12 -> RGBA8888 (CSC + 180度旋转回环)
            int r2 = pProcessOpt(nv12_b.buf, rgba_a.buf, empty_pat,
                                 full_rect, full_rect, empty_rect,
                                 -1, nullptr, &opt, kRgaUsageRot180 | kRgaUsageSync);
            if (r2 != kRgaStatusSuccess) {
                failed.store(true);
                break;
            }
        }

        free_surf(rgba_a);
        free_surf(nv12_b);
    };

    std::thread t0(rga_worker, kRgaSchedRga2Core0, 1);
    std::thread t1(rga_worker, kRgaSchedRga2Core0, 2);
    std::thread t2(rga_worker, kRgaSchedRga2Core1, 3);
    std::thread t3(rga_worker, kRgaSchedRga2Core1, 4);
    t0.join();
    t1.join();
    t2.join();
    t3.join();
    return failed.load() ? 3 : 0;
}

int run_internal_mpp_burn() {
    void* lib = dlopen("librockchip_mpp.so.1", RTLD_NOW);
    if (!lib) lib = dlopen("librockchip_mpp.so", RTLD_NOW);
    if (!lib) lib = dlopen("/usr/lib/aarch64-linux-gnu/librockchip_mpp.so.1", RTLD_NOW);
    if (!lib) lib = dlopen("/usr/lib/aarch64-linux-gnu/librockchip_mpp.so", RTLD_NOW);
    if (!lib) return 1;

    auto pCreate = reinterpret_cast<MppRet(*)(MppCtx*, MppApiTable**)>(dlsym(lib, "mpp_create"));
    auto pInit = reinterpret_cast<MppRet(*)(MppCtx, int, int)>(dlsym(lib, "mpp_init"));
    auto pDestroy = reinterpret_cast<MppRet(*)(MppCtx)>(dlsym(lib, "mpp_destroy"));

    auto pGrpGet = reinterpret_cast<MppRet(*)(MppBufferGroup*, int, int, const char*, const char*)>(dlsym(lib, "mpp_buffer_group_get"));
    auto pGrpPut = reinterpret_cast<MppRet(*)(MppBufferGroup)>(dlsym(lib, "mpp_buffer_group_put"));
    auto pGrpLimit = reinterpret_cast<MppRet(*)(MppBufferGroup, size_t, int32_t)>(dlsym(lib, "mpp_buffer_group_limit_config"));
    auto pBufGet = reinterpret_cast<MppRet(*)(MppBufferGroup, MppBuffer*, size_t, const char*, const char*)>(dlsym(lib, "mpp_buffer_get_with_tag"));
    auto pBufPut = reinterpret_cast<MppRet(*)(MppBuffer, const char*)>(dlsym(lib, "mpp_buffer_put_with_caller"));
    auto pBufPtr = reinterpret_cast<void*(*)(MppBuffer, const char*)>(dlsym(lib, "mpp_buffer_get_ptr_with_caller"));

    auto pFrmInit = reinterpret_cast<MppRet(*)(MppFrame*)>(dlsym(lib, "mpp_frame_init"));
    auto pFrmDeinit = reinterpret_cast<MppRet(*)(MppFrame*)>(dlsym(lib, "mpp_frame_deinit"));
    auto pFrmSetW = reinterpret_cast<void(*)(MppFrame, uint32_t)>(dlsym(lib, "mpp_frame_set_width"));
    auto pFrmSetH = reinterpret_cast<void(*)(MppFrame, uint32_t)>(dlsym(lib, "mpp_frame_set_height"));
    auto pFrmSetHS = reinterpret_cast<void(*)(MppFrame, uint32_t)>(dlsym(lib, "mpp_frame_set_hor_stride"));
    auto pFrmSetVS = reinterpret_cast<void(*)(MppFrame, uint32_t)>(dlsym(lib, "mpp_frame_set_ver_stride"));
    auto pFrmSetFmt = reinterpret_cast<void(*)(MppFrame, int)>(dlsym(lib, "mpp_frame_set_fmt"));
    auto pFrmSetBuf = reinterpret_cast<void(*)(MppFrame, MppBuffer)>(dlsym(lib, "mpp_frame_set_buffer"));
    auto pFrmSetEos = reinterpret_cast<void(*)(MppFrame, uint32_t)>(dlsym(lib, "mpp_frame_set_eos"));
    auto pFrmInfoChg = reinterpret_cast<uint32_t(*)(const MppFrame)>(dlsym(lib, "mpp_frame_get_info_change"));
    auto pFrmBufSize = reinterpret_cast<size_t(*)(const MppFrame)>(dlsym(lib, "mpp_frame_get_buf_size"));

    auto pPktInit = reinterpret_cast<MppRet(*)(MppPacket*, void*, size_t)>(dlsym(lib, "mpp_packet_init"));
    auto pPktInitBuf = reinterpret_cast<MppRet(*)(MppPacket*, MppBuffer)>(dlsym(lib, "mpp_packet_init_with_buffer"));
    auto pPktDeinit = reinterpret_cast<MppRet(*)(MppPacket*)>(dlsym(lib, "mpp_packet_deinit"));
    auto pPktSetLen = reinterpret_cast<void(*)(MppPacket, size_t)>(dlsym(lib, "mpp_packet_set_length"));
    auto pPktGetPos = reinterpret_cast<void*(*)(const MppPacket)>(dlsym(lib, "mpp_packet_get_pos"));
    auto pPktGetLen = reinterpret_cast<size_t(*)(const MppPacket)>(dlsym(lib, "mpp_packet_get_length"));

    auto pEncCfgInit = reinterpret_cast<MppRet(*)(MppEncCfg*)>(dlsym(lib, "mpp_enc_cfg_init"));
    auto pEncCfgDeinit = reinterpret_cast<MppRet(*)(MppEncCfg)>(dlsym(lib, "mpp_enc_cfg_deinit"));
    auto pEncCfgSetS32 = reinterpret_cast<MppRet(*)(MppEncCfg, const char*, int32_t)>(dlsym(lib, "mpp_enc_cfg_set_s32"));

    auto pDecCfgInit = reinterpret_cast<MppRet(*)(MppDecCfg*)>(dlsym(lib, "mpp_dec_cfg_init"));
    auto pDecCfgDeinit = reinterpret_cast<MppRet(*)(MppDecCfg)>(dlsym(lib, "mpp_dec_cfg_deinit"));
    auto pDecCfgSetU32 = reinterpret_cast<MppRet(*)(MppDecCfg, const char*, uint32_t)>(dlsym(lib, "mpp_dec_cfg_set_u32"));

    if (!pCreate || !pInit || !pDestroy || !pGrpGet || !pGrpPut || !pGrpLimit ||
        !pBufGet || !pBufPut || !pBufPtr || !pFrmInit || !pFrmDeinit ||
        !pFrmSetW || !pFrmSetH || !pFrmSetHS || !pFrmSetVS || !pFrmSetFmt ||
        !pFrmSetBuf || !pFrmSetEos || !pFrmInfoChg || !pFrmBufSize ||
        !pPktInit || !pPktInitBuf || !pPktDeinit || !pPktSetLen || !pPktGetPos || !pPktGetLen ||
        !pEncCfgInit || !pEncCfgDeinit || !pEncCfgSetS32 ||
        !pDecCfgInit || !pDecCfgDeinit || !pDecCfgSetU32) {
        return 2;
    }

    // 预先生成 4 帧 4K (3840x2160) NV12 共享测试纹理，避免热循环消耗 CPU 填图，纯粹拉满 VPU 硬件算力
    constexpr int kW = 3840;
    constexpr int kH = 2160;
    constexpr size_t kFrameSize = static_cast<size_t>(kW) * kH * 3 / 2;
    constexpr int kNumSrcFrames = 4;

    MppBufferGroup shared_grp = nullptr;
    if (pGrpGet(&shared_grp, kMppBufTypeDrm, kMppBufInternal, "rk_mpp", "burn") != 0 || !shared_grp) {
        return 3;
    }
    MppBuffer src_bufs[kNumSrcFrames]{};
    for (int i = 0; i < kNumSrcFrames; ++i) {
        if (pBufGet(shared_grp, &src_bufs[i], kFrameSize, "rk_mpp", "src") != 0 || !src_bufs[i]) {
            for (int j = 0; j < i; ++j) pBufPut(src_bufs[j], "src");
            pGrpPut(shared_grp);
            return 4;
        }
        auto* ptr = static_cast<uint8_t*>(pBufPtr(src_bufs[i], "src"));
        fill_nv12_pattern(ptr, kW, kH, i);
    }

    std::atomic<bool> failed{false};

    // 工具函数：创建并初始化一个 4K H.265 (回退 H.264) 硬件编码器上下文
    auto create_encoder = [&](MppCtx& enc_ctx, MppApiTable*& enc_mpi, int& coding_out) -> bool {
        coding_out = kMppCodingHevc;
        if (pCreate(&enc_ctx, &enc_mpi) != 0 || !enc_ctx || !enc_mpi) return false;
        int64_t enc_timeout = 100;
        enc_mpi->control(enc_ctx, kMppCmdSetOutputTimeout, &enc_timeout);
        if (pInit(enc_ctx, kMppCtxEnc, coding_out) != 0) {
            coding_out = kMppCodingAvc;
            if (pInit(enc_ctx, kMppCtxEnc, coding_out) != 0) {
                pDestroy(enc_ctx);
                enc_ctx = nullptr;
                return false;
            }
        }
        MppEncCfg ecfg = nullptr;
        if (pEncCfgInit(&ecfg) != 0 || enc_mpi->control(enc_ctx, kMppCmdEncGetCfg, ecfg) != 0) {
            if (ecfg) pEncCfgDeinit(ecfg);
            pDestroy(enc_ctx);
            enc_ctx = nullptr;
            return false;
        }
        pEncCfgSetS32(ecfg, "codec:type", coding_out);
        pEncCfgSetS32(ecfg, "prep:width", kW);
        pEncCfgSetS32(ecfg, "prep:height", kH);
        pEncCfgSetS32(ecfg, "prep:hor_stride", kW);
        pEncCfgSetS32(ecfg, "prep:ver_stride", kH);
        pEncCfgSetS32(ecfg, "prep:format", kMppFmtYuv420Sp);
        pEncCfgSetS32(ecfg, "rc:mode", kMppRcModeCbr);
        pEncCfgSetS32(ecfg, "rc:bps_target", 24000000);
        pEncCfgSetS32(ecfg, "rc:bps_max", 28000000);
        pEncCfgSetS32(ecfg, "rc:bps_min", 18000000);
        pEncCfgSetS32(ecfg, "rc:gop", 30);
        pEncCfgSetS32(ecfg, "rc:fps_in_flex", 0);
        pEncCfgSetS32(ecfg, "rc:fps_in_num", 60);
        pEncCfgSetS32(ecfg, "rc:fps_in_denom", 1);
        pEncCfgSetS32(ecfg, "rc:fps_out_flex", 0);
        pEncCfgSetS32(ecfg, "rc:fps_out_num", 60);
        pEncCfgSetS32(ecfg, "rc:fps_out_denom", 1);
        pEncCfgSetS32(ecfg, "rc:qp_init", 26);
        pEncCfgSetS32(ecfg, "rc:qp_min", 10);
        pEncCfgSetS32(ecfg, "rc:qp_max", 51);
        pEncCfgSetS32(ecfg, "rc:qp_min_i", 10);
        pEncCfgSetS32(ecfg, "rc:qp_max_i", 51);
        bool ok = (enc_mpi->control(enc_ctx, kMppCmdEncSetCfg, ecfg) == 0);
        pEncCfgDeinit(ecfg);
        if (!ok) {
            pDestroy(enc_ctx);
            enc_ctx = nullptr;
        }
        return ok;
    };

    // 阶段 1：启动时预编码 1 个完整 30 帧 4K H.265 GOP（首帧前拼入 VPS/SPS/PPS 头）
    // 供解码器完全解耦、零互斥锁、零分帧解析开销（split_parse=0）地满速循环硬件解码
    int dec_coding = kMppCodingHevc;
    std::vector<std::vector<uint8_t>> gop_pkts;
    {
        MppCtx init_enc = nullptr;
        MppApiTable* init_mpi = nullptr;
        if (create_encoder(init_enc, init_mpi, dec_coding)) {
            std::vector<uint8_t> hdr_bytes;
            MppBuffer hdr_buf = nullptr;
            if (pBufGet(shared_grp, &hdr_buf, 65536, "rk_mpp", "hdr") == 0 && hdr_buf) {
                MppPacket hdr_pkt = nullptr;
                if (pPktInitBuf(&hdr_pkt, hdr_buf) == 0 && hdr_pkt) {
                    pPktSetLen(hdr_pkt, 0);
                    if (init_mpi->control(init_enc, kMppCmdEncGetHdrSync, hdr_pkt) == 0) {
                        void* hp = pPktGetPos(hdr_pkt);
                        size_t hl = pPktGetLen(hdr_pkt);
                        if (hp && hl > 0) {
                            hdr_bytes.assign(static_cast<uint8_t*>(hp), static_cast<uint8_t*>(hp) + hl);
                        }
                    }
                    pPktDeinit(&hdr_pkt);
                }
                pBufPut(hdr_buf, "hdr");
            }

            for (int f = 0; f < 30 && !g_quit_flag; ++f) {
                MppFrame frm = nullptr;
                if (pFrmInit(&frm) != 0 || !frm) break;
                pFrmSetW(frm, kW);
                pFrmSetH(frm, kH);
                pFrmSetHS(frm, kW);
                pFrmSetVS(frm, kH);
                pFrmSetFmt(frm, kMppFmtYuv420Sp);
                pFrmSetEos(frm, 0);
                pFrmSetBuf(frm, src_bufs[f % kNumSrcFrames]);
                MppRet er = init_mpi->encode_put_frame(init_enc, frm);
                pFrmDeinit(&frm);
                if (er != 0) break;

                MppPacket out_pkt = nullptr;
                if (init_mpi->encode_get_packet(init_enc, &out_pkt) == 0 && out_pkt) {
                    void* pdata = pPktGetPos(out_pkt);
                    size_t plen = pPktGetLen(out_pkt);
                    if (pdata && plen > 0) {
                        std::vector<uint8_t> pkt_vec;
                        if (f == 0 && !hdr_bytes.empty()) {
                            pkt_vec.reserve(hdr_bytes.size() + plen);
                            pkt_vec.insert(pkt_vec.end(), hdr_bytes.begin(), hdr_bytes.end());
                        }
                        pkt_vec.insert(pkt_vec.end(),
                                       static_cast<uint8_t*>(pdata),
                                       static_cast<uint8_t*>(pdata) + plen);
                        gop_pkts.push_back(std::move(pkt_vec));
                    }
                    pPktDeinit(&out_pkt);
                }
            }
            init_mpi->reset(init_enc);
            pDestroy(init_enc);
        }
    }

    // 阶段 2A：独立 4K 硬件编码线程（双核 RKVENC2 Core0 + Core1 零反压满速编码）
    auto enc_worker = [&](int ch_id) {
        MppCtx enc_ctx = nullptr;
        MppApiTable* enc_mpi = nullptr;
        int coding = kMppCodingHevc;
        if (!create_encoder(enc_ctx, enc_mpi, coding)) {
            failed.store(true);
            return;
        }
        uint64_t frame_idx = static_cast<uint64_t>(ch_id);
        while (!g_quit_flag) {
            MppFrame frm = nullptr;
            if (pFrmInit(&frm) != 0 || !frm) {
                failed.store(true);
                break;
            }
            pFrmSetW(frm, kW);
            pFrmSetH(frm, kH);
            pFrmSetHS(frm, kW);
            pFrmSetVS(frm, kH);
            pFrmSetFmt(frm, kMppFmtYuv420Sp);
            pFrmSetEos(frm, 0);
            pFrmSetBuf(frm, src_bufs[frame_idx % kNumSrcFrames]);

            MppRet er = enc_mpi->encode_put_frame(enc_ctx, frm);
            pFrmDeinit(&frm);
            if (er != 0) {
                if (g_quit_flag) break;
                failed.store(true);
                break;
            }

            MppPacket out_pkt = nullptr;
            MppRet gr = enc_mpi->encode_get_packet(enc_ctx, &out_pkt);
            if (gr == 0 && out_pkt) {
                pPktDeinit(&out_pkt);
            } else if (gr != kMppErrTimeout) {
                if (g_quit_flag) break;
                failed.store(true);
                break;
            }
            ++frame_idx;
        }
        enc_mpi->reset(enc_ctx);
        pDestroy(enc_ctx);
    };

    // 阶段 2B：独立 4K 硬件解码线程（RKVDEC2 满速解耦解码，split_parse=0 + 非阻塞排空，零 5ms 空等）
    auto dec_worker = [&]() {
        if (gop_pkts.empty()) return;
        MppCtx dec_ctx = nullptr;
        MppApiTable* dec_mpi = nullptr;
        if (pCreate(&dec_ctx, &dec_mpi) != 0 || !dec_ctx || !dec_mpi) return;
        if (pInit(dec_ctx, kMppCtxDec, dec_coding) != 0) {
            pDestroy(dec_ctx);
            return;
        }
        int64_t dec_in_to = 50;
        int64_t dec_out_to = 0; // MPP_POLL_NON_BLOCK: 绝不因无输出帧空等 5ms
        dec_mpi->control(dec_ctx, kMppCmdSetInputTimeout, &dec_in_to);
        dec_mpi->control(dec_ctx, kMppCmdSetOutputTimeout, &dec_out_to);

        MppDecCfg dcfg = nullptr;
        if (pDecCfgInit(&dcfg) == 0 && dcfg) {
            if (dec_mpi->control(dec_ctx, kMppCmdDecGetCfg, dcfg) == 0) {
                pDecCfgSetU32(dcfg, "base:split_parse", 0);
                pDecCfgSetU32(dcfg, "base:fast_parse", 1);
                dec_mpi->control(dec_ctx, kMppCmdDecSetCfg, dcfg);
            }
            pDecCfgDeinit(dcfg);
        }

        MppBufferGroup dec_grp = nullptr;
        auto drain_frames = [&]() -> int {
            int drained = 0;
            while (!g_quit_flag) {
                MppFrame out_frm = nullptr;
                MppRet gr = dec_mpi->decode_get_frame(dec_ctx, &out_frm);
                if (gr != 0 || !out_frm) break;
                if (pFrmInfoChg(out_frm)) {
                    size_t bsz = pFrmBufSize(out_frm);
                    if (!dec_grp) {
                        pGrpGet(&dec_grp, kMppBufTypeDrm, kMppBufInternal, "rk_mpp", "dec");
                    }
                    if (dec_grp) {
                        pGrpLimit(dec_grp, bsz, 10);
                        dec_mpi->control(dec_ctx, kMppCmdDecSetExtBufGroup, dec_grp);
                    }
                    dec_mpi->control(dec_ctx, kMppCmdDecSetInfoChangeReady, nullptr);
                }
                pFrmDeinit(&out_frm);
                ++drained;
            }
            return drained;
        };

        size_t idx = 0;
        while (!g_quit_flag) {
            auto& pkt_data = gop_pkts[idx % gop_pkts.size()];
            MppPacket dec_pkt = nullptr;
            if (pPktInit(&dec_pkt, pkt_data.data(), pkt_data.size()) != 0 || !dec_pkt) break;

            while (!g_quit_flag) {
                MppRet pr = dec_mpi->decode_put_packet(dec_ctx, dec_pkt);
                if (pr == 0) break;
                if (drain_frames() == 0) usleep(500);
            }
            pPktDeinit(&dec_pkt);
            drain_frames();
            ++idx;
        }

        dec_mpi->reset(dec_ctx);
        pDestroy(dec_ctx);
        if (dec_grp) pGrpPut(dec_grp);
    };

    // 2 路 4K 编码（拉满 RKVENC2 Core0+Core1）+ 2 路 4K 解码（拉满 RKVDEC2），完全解耦无反压
    std::thread e0(enc_worker, 0);
    std::thread e1(enc_worker, 1);
    std::thread d0(dec_worker);
    std::thread d1(dec_worker);
    e0.join();
    e1.join();
    d0.join();
    d1.join();

    for (int i = 0; i < kNumSrcFrames; ++i) {
        pBufPut(src_bufs[i], "src");
    }
    pGrpPut(shared_grp);
    return failed.load() ? 5 : 0;
}
