#include "accel_burn.hpp"
#include "common.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <thread>

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
        // 优先使用大尺寸 FP16 矩阵乘以最大化单次计算时长，将双核 NPU 负载推至 90%~100%
        struct ShapeCfg { int32_t M, K, N, type; };
        const ShapeCfg cfgs[] = {
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
