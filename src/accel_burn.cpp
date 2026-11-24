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

constexpr int kRgaFmtRgba8888 = 0x0 << 8;
constexpr int kRgaFmtNv12 = 0xa << 8;
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

int run_internal_gpu_burn() {
    // 无头 Mesa/Panfrost GBM + EGL + OpenGL ES 2.0 离屏 FBO 着色器满载压测
    void* lib_gbm = dlopen("libgbm.so.1", RTLD_NOW);
    void* lib_egl = dlopen("libEGL.so.1", RTLD_NOW);
    void* lib_gl = dlopen("libGLESv2.so.2", RTLD_NOW);
    if (!lib_gbm || !lib_egl || !lib_gl) return 20;

    auto pGbmCreate = reinterpret_cast<void*(*)(int)>(dlsym(lib_gbm, "gbm_create_device"));
    auto pGbmDestroy = reinterpret_cast<void(*)(void*)>(dlsym(lib_gbm, "gbm_device_destroy"));

    using EGLDisplay = void*;
    using EGLConfig = void*;
    using EGLContext = void*;
    using EGLSurface = void*;
    using EGLint = int32_t;
    using EGLenum = uint32_t;
    using EGLBoolean = uint32_t;

    auto pEglGetPlatDpy = reinterpret_cast<EGLDisplay(*)(EGLenum, void*, const intptr_t*)>(dlsym(lib_egl, "eglGetPlatformDisplay"));
    auto pEglGetDpy = reinterpret_cast<EGLDisplay(*)(void*)>(dlsym(lib_egl, "eglGetDisplay"));
    auto pEglInit = reinterpret_cast<EGLBoolean(*)(EGLDisplay, EGLint*, EGLint*)>(dlsym(lib_egl, "eglInitialize"));
    auto pEglBindApi = reinterpret_cast<EGLBoolean(*)(EGLenum)>(dlsym(lib_egl, "eglBindAPI"));
    auto pEglChooseCfg = reinterpret_cast<EGLBoolean(*)(EGLDisplay, const EGLint*, EGLConfig*, EGLint, EGLint*)>(dlsym(lib_egl, "eglChooseConfig"));
    auto pEglCreateCtx = reinterpret_cast<EGLContext(*)(EGLDisplay, EGLConfig, EGLContext, const EGLint*)>(dlsym(lib_egl, "eglCreateContext"));
    auto pEglCreatePbuf = reinterpret_cast<EGLSurface(*)(EGLDisplay, EGLConfig, const EGLint*)>(dlsym(lib_egl, "eglCreatePbufferSurface"));
    auto pEglMakeCur = reinterpret_cast<EGLBoolean(*)(EGLDisplay, EGLSurface, EGLSurface, EGLContext)>(dlsym(lib_egl, "eglMakeCurrent"));
    auto pEglTerm = reinterpret_cast<EGLBoolean(*)(EGLDisplay)>(dlsym(lib_egl, "eglTerminate"));

    using GLuint = uint32_t;
    using GLint = int32_t;
    using GLenum = uint32_t;
    using GLsizei = int32_t;
    auto pCreateShader = reinterpret_cast<GLuint(*)(GLenum)>(dlsym(lib_gl, "glCreateShader"));
    auto pShaderSrc = reinterpret_cast<void(*)(GLuint, GLsizei, const char* const*, const GLint*)>(dlsym(lib_gl, "glShaderSource"));
    auto pCompileShader = reinterpret_cast<void(*)(GLuint)>(dlsym(lib_gl, "glCompileShader"));
    auto pGetShaderiv = reinterpret_cast<void(*)(GLuint, GLenum, GLint*)>(dlsym(lib_gl, "glGetShaderiv"));
    auto pCreateProg = reinterpret_cast<GLuint(*)()>(dlsym(lib_gl, "glCreateProgram"));
    auto pAttachShader = reinterpret_cast<void(*)(GLuint, GLuint)>(dlsym(lib_gl, "glAttachShader"));
    auto pLinkProg = reinterpret_cast<void(*)(GLuint)>(dlsym(lib_gl, "glLinkProgram"));
    auto pGetProgiv = reinterpret_cast<void(*)(GLuint, GLenum, GLint*)>(dlsym(lib_gl, "glGetProgramiv"));
    auto pUseProg = reinterpret_cast<void(*)(GLuint)>(dlsym(lib_gl, "glUseProgram"));
    auto pGetAttrib = reinterpret_cast<GLint(*)(GLuint, const char*)>(dlsym(lib_gl, "glGetAttribLocation"));
    auto pGetUniform = reinterpret_cast<GLint(*)(GLuint, const char*)>(dlsym(lib_gl, "glGetUniformLocation"));
    auto pUniform1f = reinterpret_cast<void(*)(GLint, float)>(dlsym(lib_gl, "glUniform1f"));
    auto pGenFB = reinterpret_cast<void(*)(GLsizei, GLuint*)>(dlsym(lib_gl, "glGenFramebuffers"));
    auto pBindFB = reinterpret_cast<void(*)(GLenum, GLuint)>(dlsym(lib_gl, "glBindFramebuffer"));
    auto pGenTex = reinterpret_cast<void(*)(GLsizei, GLuint*)>(dlsym(lib_gl, "glGenTextures"));
    auto pBindTex = reinterpret_cast<void(*)(GLenum, GLuint)>(dlsym(lib_gl, "glBindTexture"));
    auto pTexParam = reinterpret_cast<void(*)(GLenum, GLenum, GLint)>(dlsym(lib_gl, "glTexParameteri"));
    auto pTexImg2D = reinterpret_cast<void(*)(GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void*)>(dlsym(lib_gl, "glTexImage2D"));
    auto pFBTex2D = reinterpret_cast<void(*)(GLenum, GLenum, GLenum, GLuint, GLint)>(dlsym(lib_gl, "glFramebufferTexture2D"));
    auto pGenBuf = reinterpret_cast<void(*)(GLsizei, GLuint*)>(dlsym(lib_gl, "glGenBuffers"));
    auto pBindBuf = reinterpret_cast<void(*)(GLenum, GLuint)>(dlsym(lib_gl, "glBindBuffer"));
    auto pBufData = reinterpret_cast<void(*)(GLenum, ptrdiff_t, const void*, GLenum)>(dlsym(lib_gl, "glBufferData"));
    auto pEnableVA = reinterpret_cast<void(*)(GLuint)>(dlsym(lib_gl, "glEnableVertexAttribArray"));
    auto pVAPtr = reinterpret_cast<void(*)(GLuint, GLint, GLenum, uint8_t, GLsizei, const void*)>(dlsym(lib_gl, "glVertexAttribPointer"));
    auto pViewport = reinterpret_cast<void(*)(GLint, GLint, GLsizei, GLsizei)>(dlsym(lib_gl, "glViewport"));
    auto pDrawArrays = reinterpret_cast<void(*)(GLenum, GLint, GLsizei)>(dlsym(lib_gl, "glDrawArrays"));
    auto pFinish = reinterpret_cast<void(*)()>(dlsym(lib_gl, "glFinish"));

    if (!pGbmCreate || !pGbmDestroy || (!pEglGetPlatDpy && !pEglGetDpy) || !pEglInit || !pEglBindApi ||
        !pEglChooseCfg || !pEglCreateCtx || !pEglMakeCur || !pEglTerm ||
        !pCreateShader || !pShaderSrc || !pCompileShader || !pGetShaderiv ||
        !pCreateProg || !pAttachShader || !pLinkProg || !pGetProgiv || !pUseProg ||
        !pGetAttrib || !pGetUniform || !pUniform1f || !pGenFB || !pBindFB ||
        !pGenTex || !pBindTex || !pTexParam || !pTexImg2D || !pFBTex2D ||
        !pGenBuf || !pBindBuf || !pBufData || !pEnableVA || !pVAPtr ||
        !pViewport || !pDrawArrays || !pFinish) {
        return 21;
    }

    const char* dri_cands[] = {
        "/dev/dri/by-path/platform-27800000.gpu-render",
        "/dev/dri/renderD129",
        "/dev/dri/renderD128"
    };
    int drm_fd = -1;
    for (const char* p : dri_cands) {
        drm_fd = open(p, O_RDWR | O_CLOEXEC);
        if (drm_fd >= 0) break;
    }
    if (drm_fd < 0) return 22;

    void* gbm = pGbmCreate(drm_fd);
    if (!gbm) {
        close(drm_fd);
        return 23;
    }

    constexpr EGLenum kEglPlatGbm = 0x31D7;
    constexpr EGLenum kEglApiGles = 0x30A0;
    constexpr EGLint kEglRenderable = 0x3040;
    constexpr EGLint kEglEs2Bit = 0x0004;
    constexpr EGLint kEglSurfType = 0x3033;
    constexpr EGLint kEglPbufBit = 0x0001;
    constexpr EGLint kEglCtxVer = 0x3098;
    constexpr EGLint kEglWidth = 0x3057;
    constexpr EGLint kEglHeight = 0x3056;
    constexpr EGLint kEglNone = 0x3038;

    EGLDisplay dpy = pEglGetPlatDpy ? pEglGetPlatDpy(kEglPlatGbm, gbm, nullptr) : pEglGetDpy(gbm);
    if (!dpy || !pEglInit(dpy, nullptr, nullptr)) {
        pGbmDestroy(gbm);
        close(drm_fd);
        return 24;
    }
    pEglBindApi(kEglApiGles);

    EGLConfig cfg = nullptr;
    EGLint ncfg = 0;
    const EGLint cfg_attr[] = {kEglRenderable, kEglEs2Bit, kEglNone};
    if (!pEglChooseCfg(dpy, cfg_attr, &cfg, 1, &ncfg) || ncfg == 0) {
        pEglTerm(dpy);
        pGbmDestroy(gbm);
        close(drm_fd);
        return 25;
    }

    const EGLint ctx_attr[] = {kEglCtxVer, 2, kEglNone};
    EGLContext ctx = pEglCreateCtx(dpy, cfg, nullptr, ctx_attr);
    if (!ctx) {
        pEglTerm(dpy);
        pGbmDestroy(gbm);
        close(drm_fd);
        return 26;
    }

    EGLSurface surf = nullptr;
    if (!pEglMakeCur(dpy, nullptr, nullptr, ctx)) {
        const EGLint pbuf_cfg_attr[] = {kEglRenderable, kEglEs2Bit, kEglSurfType, kEglPbufBit, kEglNone};
        if (pEglCreatePbuf && pEglChooseCfg(dpy, pbuf_cfg_attr, &cfg, 1, &ncfg) && ncfg > 0) {
            const EGLint pb_attr[] = {kEglWidth, 64, kEglHeight, 64, kEglNone};
            surf = pEglCreatePbuf(dpy, cfg, pb_attr);
        }
        if (!surf || !pEglMakeCur(dpy, surf, surf, ctx)) {
            pEglTerm(dpy);
            pGbmDestroy(gbm);
            close(drm_fd);
            return 27;
        }
    }

    const char* vsrc =
        "attribute vec2 aPos;"
        "varying vec2 vUv;"
        "void main() {"
        "  vUv = aPos * 0.5 + 0.5;"
        "  gl_Position = vec4(aPos, 0.0, 1.0);"
        "}";
    const char* fsrc =
        "precision highp float;"
        "varying vec2 vUv;"
        "uniform float uPhase;"
        "void main() {"
        "  vec4 a = vec4(vUv.x + 0.37, vUv.y + 0.61, uPhase + 0.23, 0.89);"
        "  vec4 b = vec4(0.7519, 0.7523, 0.7517, 0.7521);"
        "  vec4 c = vec4(0.3712, 0.4198, 0.3356, 0.4512);"
        "  vec4 d = vec4(1.8734, 1.7923, 1.9145, 1.7456);"
        "  for (int i = 0; i < 384; ++i) {"
        "    a = a * b + c;"
        "    a = d - a * b;"
        "  }"
        "  gl_FragColor = clamp(a * 0.5, 0.0, 1.0);"
        "}";

    GLuint vs = pCreateShader(0x8B31);
    pShaderSrc(vs, 1, &vsrc, nullptr);
    pCompileShader(vs);
    GLint ok = 0;
    pGetShaderiv(vs, 0x8B81, &ok);
    if (!ok) return 28;

    GLuint fs = pCreateShader(0x8B30);
    pShaderSrc(fs, 1, &fsrc, nullptr);
    pCompileShader(fs);
    pGetShaderiv(fs, 0x8B81, &ok);
    if (!ok) return 29;

    GLuint prog = pCreateProg();
    pAttachShader(prog, vs);
    pAttachShader(prog, fs);
    pLinkProg(prog);
    pGetProgiv(prog, 0x8B82, &ok);
    if (!ok) return 30;
    pUseProg(prog);

    // 1024x1024 离屏 FBO（100 万像素 x 384 次双 vec4 FMA/帧，直接将 Mali-G52 推至 100% 满频）
    constexpr GLsizei kDim = 1024;
    GLuint tex = 0, fbo = 0, vbo = 0;
    pGenTex(1, &tex);
    pBindTex(0x0DE1, tex);
    pTexParam(0x0DE1, 0x2801, 0x2600);
    pTexParam(0x0DE1, 0x2800, 0x2600);
    pTexImg2D(0x0DE1, 0, 0x1908, kDim, kDim, 0, 0x1908, 0x1401, nullptr);
    pGenFB(1, &fbo);
    pBindFB(0x8D40, fbo);
    pFBTex2D(0x8D40, 0x8CE0, 0x0DE1, tex, 0);

    static const float quad[12] = {
        -1.0f, -1.0f,  1.0f, -1.0f,  -1.0f,  1.0f,
         1.0f, -1.0f,  1.0f,  1.0f,  -1.0f,  1.0f
    };
    pGenBuf(1, &vbo);
    pBindBuf(0x8892, vbo);
    pBufData(0x8892, sizeof(quad), quad, 0x88E4);
    GLint pos_loc = pGetAttrib(prog, "aPos");
    GLint phase_loc = pGetUniform(prog, "uPhase");
    if (pos_loc >= 0) {
        pEnableVA(static_cast<GLuint>(pos_loc));
        pVAPtr(static_cast<GLuint>(pos_loc), 2, 0x1406, 0, 0, nullptr);
    }
    pViewport(0, 0, kDim, kDim);

    float phase = 0.1f;
    while (!g_quit_flag) {
        for (int r = 0; r < 4; ++r) {
            if (phase_loc >= 0) pUniform1f(phase_loc, phase);
            pDrawArrays(0x0004, 0, 6);
            phase += 0.013f;
            if (phase > 1.0f) phase -= 1.0f;
        }
        pFinish();
    }

    pEglMakeCur(dpy, nullptr, nullptr, nullptr);
    pEglTerm(dpy);
    pGbmDestroy(gbm);
    close(drm_fd);
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
    auto pProcess = reinterpret_cast<int(*)(RgaBuffer, RgaBuffer, RgaBuffer, RgaRect, RgaRect, RgaRect, int)>(dlsym(lib, "improcess"));
    if (!pImportFd || !pImportVa || !pRelease || !pWrapHandle || !pProcess) return 2;

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

    // 启动 4 路并发工作线程，由内核 RGA 多核调度器自动均衡打满双核 RGA2 至 95%~100%
    std::atomic<bool> failed{false};
    auto rga_worker = [&](uint8_t seed) {
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

        while (!g_quit_flag) {
            // RGBA8888 -> NV12 (CSC + 180度旋转)
            int r1 = pProcess(rgba_a.buf, nv12_b.buf, empty_pat,
                              full_rect, full_rect, empty_rect,
                              kRgaUsageRot180 | kRgaUsageSync);
            if (r1 != kRgaStatusSuccess) {
                failed.store(true);
                break;
            }
            if (g_quit_flag) break;
            // NV12 -> RGBA8888 (CSC + 180度旋转回环)
            int r2 = pProcess(nv12_b.buf, rgba_a.buf, empty_pat,
                              full_rect, full_rect, empty_rect,
                              kRgaUsageRot180 | kRgaUsageSync);
            if (r2 != kRgaStatusSuccess) {
                failed.store(true);
                break;
            }
        }

        free_surf(rgba_a);
        free_surf(nv12_b);
    };

    std::thread t0(rga_worker, 1);
    std::thread t1(rga_worker, 2);
    std::thread t2(rga_worker, 3);
    std::thread t3(rga_worker, 4);
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
