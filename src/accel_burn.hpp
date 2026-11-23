#pragma once

// 内置原生硬件 GPU (Mali OpenCL) & NPU (RKNPU 双核 MatMul) 压测工作进程入口
// 通过 fork() + execl("/proc/self/exe", "rk_tester", "--internal-*-burn", nullptr) 启动
int run_internal_gpu_opencl_burn();
int run_internal_npu_matmul_burn();
