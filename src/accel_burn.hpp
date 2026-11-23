#pragma once

// 内置原生硬件压测工作进程入口：
// - GPU  : Mali-G52 OpenCL 计算压测 (--internal-gpu-burn)
// - NPU  : RKNPU 双核 FP16 MatMul 压测 (--internal-npu-burn)
// - RGA  : RGA2 双核 4K 2D CSC/旋转/缩放压测 (--internal-rga-burn)
// - RKMPP: 双通道 4K H.265 VENC+VDEC 编解码回环压测 (--internal-mpp-burn)
// 均通过 fork() + execl("/proc/self/exe", "rk_tester", "--internal-*-burn", nullptr) 启动
int run_internal_gpu_opencl_burn();
int run_internal_npu_matmul_burn();
int run_internal_rga_burn();
int run_internal_mpp_burn();
