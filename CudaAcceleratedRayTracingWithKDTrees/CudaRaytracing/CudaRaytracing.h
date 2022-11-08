#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <vector_types.h>
#include <device_launch_parameters.h>
#include <limits>
#include "../triangle.h"
#include "../kdnode.h"
#include "../vector.h"
#include "../ray.h"
#include "../image.h"


struct cuda_exception : std::exception {
    const cudaError_t code;
    cuda_exception(cudaError_t code) : code(code), std::exception() {
    }

    const char* what() const throw() {
        return cudaGetErrorString(code);
    }
};

#define CUERR { \
cudaError_t err; \
if ((err = cudaGetLastError()) != cudaSuccess) { \
std::cerr << "CUDA error: " << cudaGetErrorString(err) << " : " \
<< __FILE__ << ", line " << __LINE__ << std::endl; \
throw cuda_exception(err); \
} \
}

#define gpuErrchk(ans) do { gpuAssert((ans), __FILE__, __LINE__);} while(0)
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true)
{
    if (code != cudaSuccess)
    {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) throw cuda_exception(code);
    }
}

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;



void cudaNaiveTrace(Triangle*, Image&, int);

void cudaKDTrace(KDNode*, Image&, Triangle*, int*, int, int, int, Point, Point);


class GPUHit
{
public:
    double t;
    float3 N;
    int hitobject;
    bool no_hit;

    __device__ GPUHit();

    __device__ GPUHit(double t, const float3& normal, int hitobject = -1, bool nohit = false)
        : t(t), N(normal), hitobject(hitobject), no_hit(nohit)
    { }

    //__device__ static const GPUHit NO_HIT() { static GPUHit no_hit(__builtin_nan("0"), float3{ __builtin_nan("0"), __builtin_nan("0"), __builtin_nan("0") }, -1, true); return no_hit; }

};
