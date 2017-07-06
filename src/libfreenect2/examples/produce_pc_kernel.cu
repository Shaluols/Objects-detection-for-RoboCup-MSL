 #include <cuda.h>
 #include <cuda_runtime.h>
 #include "device_functions.h"
 #include "produce_pc_kernel.h"
 #include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#ifndef __CUDACC__
#define __CUDACC__
#endif
#include "device_launch_parameters.h"
#include <cuda_runtime_api.h>
#include <string.h> // memcpy
#include <cstdlib>
#include <signal.h>
#include <time.h>

texture<int, 1, cudaReadModeElementType> texref0, texref2;
texture<float, 1, cudaReadModeElementType> texref1;
const size_t imageSize1 = 4 * 424 * 512;
__global__ void kernel2(const float* _depth, const float* _rgba, const int _width, const int _height, const float* cc, float *_cloud)//(const float* _depth, const float* _rgb, const int _width, const int _height, const int table_x[], const int table_y[], float *_cloud)
{

    int c = threadIdx.x + blockIdx.x * blockDim.x;//0--511
    int r = threadIdx.y + blockIdx.y * blockDim.y;//0-423

    const int index = (c) + (r) * _width;//0--512*424-1=217087
    const float cx = cc[0];
    const float cy = cc[1];
    const float fx = cc[2];
    const float fy = cc[3];
    if ( c < _width )
    {
        if ( r < _height)
        {
            const float &depth_v = _depth[index]; //scaling factor, so that value of 1 is one meter.
            _cloud[index*8+0] = (c-cx) * (fx) * depth_v;
            _cloud[index*8+1] = (r-cy) * (fy) * depth_v;
            _cloud[index*8+2] = depth_v;
            _cloud[index*8+4] = _rgba[index];
        }
     }
    __syncthreads();
  }

__global__ void kernel3(const float* d_dis_depth, const unsigned int* d_raw_rgb, unsigned int* d_register, const float* cc, float *_cloud)//, const float* cc, float *_cloud
{
    int c = threadIdx.x + blockIdx.x * blockDim.x;//0--511
    int r = threadIdx.y + blockIdx.y * blockDim.y;//0-423
    const int index = (c) + (r) * 512;//0--512*424-1=217087
    int depth_to_c_off[512*424];
//    int offset = threadIdx.x + threadIdx.y * 32;
    const float &cx = cc[0];
    const float &cy = cc[1];
    const float &fx = cc[2];
    const float &fy = cc[3];
    if ( r < 424 )
    {
        if ( c < 512)
        {
            const int &id = tex1Dfetch(texref0,index);
            const float z = id <0 ? 0.02f : d_dis_depth[id];
            const int c_off = (z>0)*((tex1Dfetch(texref1,index) + __fdividef(52.0f,z)) * 1081.372070f + 960.0f + tex1Dfetch(texref2,index) * 1920);
            depth_to_c_off[index] = ((z>0)&&(id>=0)&&(c_off>=0)&&(c_off<1920 * 1080)) * (c_off+1) + (-1);
            const float &depth_v = d_dis_depth[id] * (id>=0);
            _cloud[index*8+0] = (c-cx) * (fx) * depth_v;
            _cloud[index*8+1] = (r-cy) * (fy) * depth_v;
            _cloud[index*8+2] = depth_v;
            const int &rgb_val = depth_to_c_off[index] < 0 ? 0 : d_raw_rgb[depth_to_c_off[index]];

            d_register[index] = rgb_val;

            u_char* rgba = (u_char*) ( _cloud + index*8+4 );
            const u_char* bgra = (const u_char*) &rgb_val;
            rgba[0] = bgra[2];
            rgba[1] = bgra[1];
            rgba[2] = bgra[0];
        }
    }
    __syncthreads();
}

void registration_kernel(const float* d_dis_depth, const unsigned int* d_raw_rgb, unsigned int* d_register, const int* d_map_dist, const float* d_map_x, const int* d_map_yi, const float* cc, float *_cloud)//, const float* cc, float *_cloud
{
    cudaBindTexture(0,texref0,d_map_dist,imageSize1);
    cudaBindTexture(0,texref1,d_map_x,imageSize1);
    cudaBindTexture(0,texref2,d_map_yi,imageSize1);
    dim3 threadsPerBlock(32, 4);
    dim3 Grid((512+threadsPerBlock.x-1)/threadsPerBlock.x, (424+threadsPerBlock.y-1)/threadsPerBlock.y);
    kernel3<<< Grid, threadsPerBlock >>>(d_dis_depth, d_raw_rgb, d_register, cc, _cloud);
    cudaDeviceSynchronize();
    cudaThreadSynchronize();
    cudaUnbindTexture(texref0);
    cudaUnbindTexture(texref1);
    cudaUnbindTexture(texref2);

}
void produce_pc_kernel2(const float* _depth, const float* _rgba, const int _width, const int _height, const float* c, float *_cloud)//(const float* _depth, const float* _rgb, const int _width, const int _height, const int table_x[], const int table_y[], float *_cloud)
{
    dim3 threadsPerBlock(32, 8);
    dim3 Grid((_width+threadsPerBlock.x-1)/threadsPerBlock.x, (_height+threadsPerBlock.y-1)/threadsPerBlock.y);
    kernel2<<< Grid, threadsPerBlock >>>(_depth, _rgba, _width, _height, c, _cloud);
}
