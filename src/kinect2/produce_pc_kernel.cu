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
#include <math.h>
#include <thrust/device_ptr.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <thrust/sort.h>
#include <thrust/device_vector.h>
//#include "Protonect.h"
//#include <af/cuda.h>
//#include <arrayfire.h>
#include <opencv2/opencv.hpp>
#include <LaterMethods.h>
#define PI 3.1415

using namespace std;
texture<int, 1, cudaReadModeElementType> texref0, texref2;
texture<float, 1, cudaReadModeElementType> texref1;
const size_t imageSize1 = 4 * 424 * 512;
dim3 threadsPerBlock(32, 32);
dim3 threadsPerBlock1(1, 400);
dim3 Grid1((400+threadsPerBlock1.x-1)/threadsPerBlock1.x, (400+threadsPerBlock1.y-1)/threadsPerBlock1.y);
dim3 Grid_((400+threadsPerBlock.x-1)/threadsPerBlock.x, (240+threadsPerBlock.y-1)/threadsPerBlock.y);
dim3 Grid((512+threadsPerBlock.x-1)/threadsPerBlock.x, (424+threadsPerBlock.y-1)/threadsPerBlock.y);

struct is_zero
{
    __host__ __device__ bool operator()(const int x)
    {
        return (x==0);
    }
};

__global__ void peng_zhang(unsigned char * pj,unsigned char *temp_pj,size_t width, size_t height)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//0--511
    int j = threadIdx.y + blockIdx.y * blockDim.y;//0-423
    const int idex = (i) + (j) * width;//0--512*424-1=217087
    int neighbors[8] = {(i-1) + (j-1) * width, (i) + (j-1) * width,(i+1) + (j-1) * width,(i-1) + (j) * width, (i+1) + (j) * width,(i-1) + (j+1) * width,(i) + (j+1) * width,(i+1) + (j+1) * width};
    if ( j < height )
    {
         if ( i < width)
        {
            if (temp_pj[idex]==0)//目标像素
            {
                if (j==0)//上边界
                {
                    if(i==0)
                    {
                        if ((temp_pj[neighbors[4]]>0)||(temp_pj[neighbors[6]]>0)||(temp_pj[neighbors[7]]>0))//周围有不是黄色像素的
                        {
                            pj[idex]=1;

                        }
                    }
                    if(i==width-1)
                    {
                        if ((temp_pj[neighbors[3]]>0)||(temp_pj[neighbors[5]]>0)||(temp_pj[neighbors[6]]>0))//周围有不是黄色像素的
                        {
                            pj[idex]=1;
                        }
                    }
                    if((i>0) && (i<width-1))
                    {
                        int num_=0;
                        for (int k=3;k<8;k++)
                        {
                            if (temp_pj[neighbors[k]]>0)
                            {
                                num_++;
                                if (num_>2)
                                {
                                    pj[idex]=1;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (j==(height-1))//上边界
                {
                    if(i==0)
                    {
                        if ((temp_pj[neighbors[1]]>0)||(temp_pj[neighbors[2]]>0)||(temp_pj[neighbors[4]]>0))//周围有不是黄色像素的
                        {
                            pj[idex]=1;
                        }
                    }
                    if(i==width-1)
                    {
                        if ((temp_pj[neighbors[0]]>0)||(temp_pj[neighbors[1]]>0)||(temp_pj[neighbors[3]]>0))//周围有不是黄色像素的
                        {
                            pj[idex]=1;
                        }
                    }
                    if((i>0) && (i<width-1))
                    {
                        int num_=0;
                        for (int k=0;k<5;k++)
                        {
                            if (temp_pj[neighbors[k]]>0)
                            {
                                num_++;
                                if (num_>2)
                                {
                                    pj[idex]=1;
                                    break;
                                }
                            }
                        }
                    }
                }

                if ((j>0)&&(j<(height-1)))//上边界
                {
                    if(i==0)
                    {
                        if ((temp_pj[neighbors[1]]>0)||(temp_pj[neighbors[2]]>0)||(temp_pj[neighbors[4]]>0)||(temp_pj[neighbors[6]]>0)||(temp_pj[neighbors[7]]>0))//周围有不是黄色像素的
                        {
                            pj[idex]=1;
                        }
                    }
                    if(i==width-1)
                    {
                        if ((temp_pj[neighbors[0]]>0)||(temp_pj[neighbors[1]]>0)||(temp_pj[neighbors[3]]>0)||(temp_pj[neighbors[5]]>0)||(temp_pj[neighbors[6]]>0))//周围有不是黄色像素的
                        {
                            pj[idex]=1;
                        }
                    }
                    if((i>0) && (i<width-1))
                    {
                        int num_=0;
                        for (int k=0;k<8;k++)
                        {
                            if (temp_pj[neighbors[k]]>0)
                            {
                                num_++;
                                if (num_>2)
                                {
                                    pj[idex]=1;
                                    break;
                                }
                            }
                        }
                    }
                }

            }

        }//for i=512
    }//for j=424

}
__global__ void fu_shi(unsigned char * pj, unsigned char * temp_pj, size_t width, size_t height)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//0--511
    int j = threadIdx.y + blockIdx.y * blockDim.y;//0-423
    const int idex = (i) + (j) * width;//0--512*424-1=217087
    int neighbors[8] = {(i-1) + (j-1) * width, (i) + (j-1) * width,(i+1) + (j-1) * width,(i-1) + (j) * width, (i+1) + (j) * width,(i-1) + (j+1) * width,(i) + (j+1) * width,(i+1) + (j+1) * width};

    if ( j < height )
    {
         if ( i < width)
        {
            if (temp_pj[idex]>0)//目标像素
            {
                if (j==0)//上边界
                {
                    if(i==0)
                    {
                        if ((temp_pj[neighbors[4]]==0)||(temp_pj[neighbors[6]]==0)||(temp_pj[neighbors[7]]==0))//周围有不是黄色像素的
                        {
                            pj[idex]=0;
                        }
                    }
                    if(i==width-1)
                    {
                        if ((temp_pj[neighbors[3]]==0)||(temp_pj[neighbors[5]]==0)||(temp_pj[neighbors[6]]==0))//周围有不是黄色像素的
                        {
                            pj[idex]=0;
                        }
                    }
                    if((i>0) && (i<width-1))
                    {
                        int num_=0;
                        for (int k=3;k<8;k++)
                        {
                            if (temp_pj[neighbors[k]]==0)
                            {
                                num_++;
                                if (num_>2)
                                {
                                    pj[idex]=0;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (j==(height-1))//上边界
                {
                    if(i==0)
                    {
                        if ((temp_pj[neighbors[1]]==0)||(temp_pj[neighbors[2]]==0)||(temp_pj[neighbors[4]]==0))//周围有不是黄色像素的
                        {
                            pj[idex]=0;
                        }
                    }
                    if(i==width-1)
                    {
                        if ((temp_pj[neighbors[0]]==0)||(temp_pj[neighbors[1]]==0)||(temp_pj[neighbors[3]]==0))//周围有不是黄色像素的
                        {
                            pj[idex]=0;
                        }
                    }
                    if((i>0) && (i<width-1))
                    {
                        int num_=0;
                        for (int k=0;k<5;k++)
                        {
                            if (temp_pj[neighbors[k]]==0)
                            {
                                num_++;
                                if (num_>2)
                                {
                                    pj[idex]=0;
                                    break;
                                }
                            }
                        }
                    }
                }

                if ((j>0)&&(j<(height-1)))//上边界
                {
                    if(i==0)
                    {
                        if ((temp_pj[neighbors[1]]==0)||(temp_pj[neighbors[2]]==0)||(temp_pj[neighbors[4]]==0)||(temp_pj[neighbors[6]]==0)||(temp_pj[neighbors[7]]==0))//周围有不是黄色像素的
                        {
                            pj[idex]=1;
                        }
                    }
                    if(i==width-1)
                    {
                        if ((temp_pj[neighbors[0]]==0)||(temp_pj[neighbors[1]]==0)||(temp_pj[neighbors[3]]==0)||(temp_pj[neighbors[5]]==0)||(temp_pj[neighbors[6]]==0))//周围有不是黄色像素的
                        {
                            pj[idex]=0;
                        }
                    }
                    if((i>0) && (i<width-1))
                    {
                        int num_=0;
                        for (int k=0;k<8;k++)
                        {
                            if (temp_pj[neighbors[k]]==0)
                            {
                                num_++;
                                if (num_>2)
                                {
                                    pj[idex]=0;
                                    break;
                                }
                            }
                        }
                    }
                }

            }

        }//for i=512
    }//for j=424

            //腐蚀代码


}
__global__ void kernel3(const float* d_dis_depth, const unsigned int* d_raw_rgb, const float* cc, float *_cloud,unsigned char *d_table, unsigned char *seg_result,int *n)//, const float* cc, float *_cloud
{
    int c = threadIdx.x + blockIdx.x * blockDim.x;//0--511
    int r = threadIdx.y + blockIdx.y * blockDim.y;//0-423
    const int index = (c) + (r) * 512;//0--512*424-1=217087
    int depth_to_c_off;
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
            depth_to_c_off = ((z>0)&&(id>=0)&&(c_off>=0)&&(c_off<1920 * 1080)) * (c_off+1) + (-1);
//            const float &depth_v = d_dis_depth[id] * (id>=0);
//            _cloud[index*8+0] = (c-cx) * (fx) * depth_v;
//            _cloud[index*8+1] = (r-cy) * (fy) * depth_v;
//            _cloud[index*8+2] = depth_v;
            const float &depth_v = (id>=0) * d_dis_depth[id];
            _cloud[index*8+0] = ((depth_v>12000.f)||(depth_v<=1.f))? 100000.0f : ((c-cx) * (fx) * depth_v);
            _cloud[index*8+1] = ((depth_v>12000.f)||(depth_v<=1.f))? 100000.0f : ((r-cy) * (fy) * depth_v);
            _cloud[index*8+2] = ((depth_v>12000.f)||(depth_v<=1.f))? 100000.0f : depth_v;
            const int &rgb_val = depth_to_c_off < 0 ? 0 : d_raw_rgb[depth_to_c_off];
            u_char* rgba = (u_char*) ( _cloud + index*8+4 );
            const u_char* bgra = (const u_char*) &rgb_val;
            rgba[0] = bgra[2];
            rgba[1] = bgra[1];
            rgba[2] = bgra[0];
            //颜色分割
            seg_result[index]=d_table[rgba[0]/4*64*64 + rgba[1]/4*64 + rgba[2]/4];
            if(seg_result[index]==0)//黄色像素
            {
                seg_result[index]=1;
                atomicAdd(&(n[0]),1);
            }
            else
                seg_result[index]=0;
        }
    }
    __syncthreads();
}

void registration_kernel(const float* d_dis_depth, const unsigned int* d_raw_rgb, const int* d_map_dist, const float* d_map_x, const int* d_map_yi, const float* cc, float *_cloud, unsigned char *d_table, unsigned char *seg_result, int *num_yellow)//, const float* cc, float *_cloud
{
    cudaBindTexture(0,texref0,d_map_dist,imageSize1);
    cudaBindTexture(0,texref1,d_map_x,imageSize1);
    cudaBindTexture(0,texref2,d_map_yi,imageSize1);

    int *d_n;
    cudaMalloc((void**)&d_n,sizeof(int));
    cudaMemset(d_n,0,sizeof(int));
    kernel3<<< Grid, threadsPerBlock >>>(d_dis_depth, d_raw_rgb, cc, _cloud, d_table, seg_result, d_n);

    cudaUnbindTexture(texref0);
    cudaUnbindTexture(texref1);
    cudaUnbindTexture(texref2);
    cudaMemcpy(num_yellow,d_n,sizeof(int),cudaMemcpyDeviceToHost);
    cudaFree(d_n);
}
//float badpt = std::numeric_limits<float>::quiet_NaN ();

__global__ void cluster_center_kernel(unsigned char *d_seg_t,unsigned char * segment_table)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//0--511
    int j = threadIdx.y + blockIdx.y * blockDim.y;//0-423
    const int idex = (i) + (j) * 512;//0--512*424-1=217087
    int neighbors[8] = {(i-1) + (j-1) * 512, (i) + (j-1) * 512,(i+1) + (j-1) * 512,(i-1) + (j) * 512, (i+1) + (j) * 512,(i-1) + (j+1) * 512,(i) + (j+1) * 512,(i+1) + (j+1) * 512};
    if ( j < 424 )
    {
        if ( i < 512)
        {
            //膨胀代码
            if (segment_table[idex]==0)//黄色像素
            {
                for (int k=0; k<8; k++)
                {
                    if ((neighbors[k]>0) && (neighbors[k]<512*424) && (segment_table[neighbors[k]]==1))//周围有不是黄色像素的
                    {
                        d_seg_t[idex]=1;
                        break;
                    }
                }
            }

        }//for i=512
    }//for j=424

}
__global__ void cluster_center_kernel_ex(unsigned char * d_seg_t,unsigned char * segment_table)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//0--511
    int j = threadIdx.y + blockIdx.y * blockDim.y;//0-423
    const int idex = (i) + (j) * 512;//0--512*424-1=217087
    int neighbors[8] = {(i-1) + (j-1) * 512, (i) + (j-1) * 512,(i+1) + (j-1) * 512,(i-1) + (j) * 512, (i+1) + (j) * 512,(i-1) + (j+1) * 512,(i) + (j+1) * 512,(i+1) + (j+1) * 512};
    if ( j < 424 )
    {
        if ( i < 512)
        {
            //腐蚀代码
            if (segment_table[idex]==1)//黄色像素
            {
                for (int k=0; k<8; k++)
                {
                    if ((neighbors[k]>0) && (neighbors[k]<512*424) && (segment_table[neighbors[k]]==0))//周围有不是黄色像素的
                    {
                        d_seg_t[idex]=0;
                        break;
                    }
                }
            }

        }//for i=512
    }//for j=424

}





__global__ void label_initialize(unsigned char * segment_table,  int *d_label, size_t width, size_t height)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
     int idex = (i) + (j) * width;//0--512*424-1=217087
    if (idex<width*height)
    {

            if (segment_table[idex]>0  && i>0)
            {
                d_label[idex]=i;
//                atomicAdd(&(ch1[0]),1);
            }
            else
                d_label[idex]=0;
            __syncthreads();
    }
}

__global__ void eight_DLS(unsigned char * segment_table, int *d_label, int *flag)//flag初始值是0，
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 512;//0--512*424-1=217087
    int neighbors[8] = {(i-1) + (j-1) * 512, (i) + (j-1) * 512,(i+1) + (j-1) * 512,(i-1) + (j) * 512, (i+1) + (j) * 512,(i-1) + (j+1) * 512,(i) + (j+1) * 512,(i+1) + (j+1) * 512};
    int no_neigbor=0;
    if (i<512)
    {
        if (j<424)
        {
            if (segment_table[idex]>0)//object pixel
            {
//               atomicAdd(&(ch[0]),1);
                int mini=d_label[idex];
                for (int n=0;n<8;n++)//对8邻域进行搜索
                {
                    while ((neighbors[n]>0)&&(neighbors[n]<512*424)&&(segment_table[neighbors[n]]!=0))//在n方向没有遇到0
                    {
                        //处理某一个方向的pixel
                        if(d_label[neighbors[n]]<mini)
                        {
                            mini=d_label[neighbors[n]];
                        }
                        switch (n) {
                        case 0://左上角
                            neighbors[n]=neighbors[n]-512-1;
                            break;
                        case 1://正上方
                            neighbors[n]=neighbors[n]-512;
                            break;
                        case 2://右上角
                            neighbors[n]=neighbors[n]-512+1;
                            break;
                        case 3://左上角
                            neighbors[n]=neighbors[n]-1;
                            break;
                        case 4://左上角
                            neighbors[n]=neighbors[n]+1;
                            break;
                        case 5://左上角
                            neighbors[n]=neighbors[n]+512-1;
                            break;
                        case 6://左上角
                            neighbors[n]=neighbors[n]+512;
                            break;
                        case 7://左上角
                            neighbors[n]=neighbors[n]+512+1;
                            break;
                        default:
                            break;
                        }
                    }

                }
                if (mini<d_label[idex])
                {
                d_label[idex]=mini;
                flag[0]=1;
                }
            }

//            __syncthreads();
        }
    }

}
__global__ void eight_DLS_last(unsigned char * segment_table,  int *d_label, int *array, int *flag, size_t nnn)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 512;//0--512*424-1=217087
    int s_idex=threadIdx.x +blockDim.x*threadIdx.y;
    int neighbors[8] = {(i-1) + (j-1) * 512, (i) + (j-1) * 512,(i+1) + (j-1) * 512,(i-1) + (j) * 512, (i+1) + (j) * 512,(i-1) + (j+1) * 512,(i) + (j+1) * 512,(i+1) + (j+1) * 512};
//    if (idex==0)
//       printf("aaaaaa %d \n",flag[0]);
    if (i<512)
    {
        if (j<424)
        {
            if ((segment_table[idex]>0) && (d_label[idex]>0))//object pixel
            {
//               if ((nnn>2)&&(d_label[d_label[idex]]==d_label[idex]))
//               {

//               }
//               else
//               {
                int mini=d_label[idex];
                for (int n=0;n<8;n++)//对8邻域进行搜索
                {
                    while ((neighbors[n]>0)&&(neighbors[n]<512*424)&&(segment_table[neighbors[n]]!=0))//在n方向没有遇到0
                    {
                        //处理某一个方向的pixel
                        if((d_label[neighbors[n]]<mini)&&(d_label[neighbors[n]]>0))
                        {
                            mini=d_label[neighbors[n]];
                        }
                        switch (n) {
                        case 0://左上角
                            neighbors[n]=neighbors[n]-512-1;
                            break;
                        case 1://正上方
                            neighbors[n]=neighbors[n]-512;
                            break;
                        case 2://右上角
                            neighbors[n]=neighbors[n]-512+1;
                            break;
                        case 3://左上角
                            neighbors[n]=neighbors[n]-1;
                            break;
                        case 4://左上角
                            neighbors[n]=neighbors[n]+1;
                            break;
                        case 5://左上角
                            neighbors[n]=neighbors[n]+512-1;
                            break;
                        case 6://左上角
                            neighbors[n]=neighbors[n]+512;
                            break;
                        case 7://左上角
                            neighbors[n]=neighbors[n]+512+1;
                            break;
                        default:
                            break;
                        }
                    }

                }
                if (mini<d_label[idex])
                {
                d_label[idex]=mini;
//                atomicExch(&(flag[0]),1);
                }
//               }//if smalllest CCL
            }//if object pixel

                    __syncthreads();
                    __shared__ bool lockx1;
                    __threadfence();
                    if(s_idex==0)
                    {
                        unsigned int lockiii1=atomicAdd(&(array[5]),1);
                        lockx1=(array[5]==224);
                    }
                    __syncthreads();
                    if(lockx1)//保证所有的块均计算完了
                    {

                    }
                    if (nnn==2)
                    {
                    if (d_label[idex]>0)
                    {
                        if ((d_label[idex]!=array[0])&&(d_label[idex]!=array[1])&&(d_label[idex]!=array[2])&&(d_label[idex]!=array[3])&&(d_label[idex]!=array[4]))
                        {
                            for (int k=0;k<5;k++)
                            {
                                if (array[k]==0)
                                {
                                    atomicExch(&(array[k]),d_label[idex]);
                                    break;
                                }
                             }
                         }
                    }

                    }

        }//j=424
    }//i=512

}//函数域
__global__ void eight_DLS_last_obstacle(unsigned char * segment_table,  int *d_label,  size_t width, size_t height)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * width;//0--512*424-1=217087
    int s_idex=threadIdx.x +blockDim.x*threadIdx.y;
    int neighbors[8] = {(i-1) + (j-1) * width, (i) + (j-1) * width,(i+1) + (j-1) * width,(i-1) + (j) * width, (i+1) + (j) * width,(i-1) + (j+1) * width,(i) + (j+1) * width,(i+1) + (j+1) * width};
//    if (idex==0)
//       printf("aaaaaa %d \n",flag[0]);
    if (i<width)
    {
        if (j<height)
        {
            if ((segment_table[idex]>0)&&(d_label[idex]>0))//object pixel
            {
//               if ((nnn>2)&&(d_label[d_label[idex]]==d_label[idex]))
//               {

//               }
//               else
//               {
                int mini=d_label[idex];

                for (int n=0;n<8;n++)//对8邻域进行搜索
                {
                    while ((neighbors[n]>0)&&(neighbors[n]<width*height)&&(segment_table[neighbors[n]]!=0))//在n方向没有遇到0
                    {
                        //处理某一个方向的pixel
                        if((d_label[neighbors[n]]<mini)&&(d_label[neighbors[n]]>0))
                        {
                            mini=d_label[neighbors[n]];

                        }
                        switch (n) {
                        case 0://左上角
                            neighbors[n]=neighbors[n]-width-1;
                            break;
                        case 1://正上方
                            neighbors[n]=neighbors[n]-width;
                            break;
                        case 2://右上角
                            neighbors[n]=neighbors[n]-width+1;
                            break;
                        case 3://左上角
                            neighbors[n]=neighbors[n]-1;
                            break;
                        case 4://左上角
                            neighbors[n]=neighbors[n]+1;
                            break;
                        case 5://左上角
                            neighbors[n]=neighbors[n]+width-1;
                            break;
                        case 6://左上角
                            neighbors[n]=neighbors[n]+width;
                            break;
                        case 7://左上角
                            neighbors[n]=neighbors[n]+width+1;
                            break;
                        default:
                            break;
                        }
                    }

                }
                if (mini<d_label[idex])
                {
                d_label[idex]=mini;
//                flag[0]=1;
                }
//               }//if smalllest CCL
            }//if object pixel

        }//j=424
    }//i=512

}//函数域
__global__ void count(int *d_label, int numb, int *center, size_t width, size_t height)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * width;//0--512*424-1=217087
    if (idex<width*height)
    {
//        if (idex==0)
//            printf("nnnnnnnnnnnnnnnnnn1111 %d %d\n\n",n[0],numb);
        if (d_label[idex]==numb)
        {
            atomicAdd(&(center[0]),1);
            atomicAdd(&(center[1]),i);
            atomicAdd(&(center[2]),j);
        }
        __syncthreads();
    }
}
double gettime()
{
    double tseconds=0.0;
    struct timeval mytime;
    gettimeofday(&mytime,(struct timezone *)0);
    tseconds=(double)(mytime.tv_sec+mytime.tv_usec*1.0e-6);
    return tseconds;
}
int CCL(unsigned char *d_seg_table, int *d_label, float *ball_position)
{
    double ti=gettime();
    unsigned char *d_seg_table_temp;
    cudaMalloc((void **)&d_seg_table_temp,512*424*sizeof(unsigned char));

    for (int j=0;j<3;j++)
    {//corrosion
        cudaMemcpy(d_seg_table_temp,d_seg_table,512*424*sizeof(unsigned char),cudaMemcpyDeviceToDevice);
        fu_shi<<< Grid, threadsPerBlock >>>(d_seg_table,d_seg_table_temp,512,424);
    }
    for (int i=0;i<4;i++)
    {//dilate
        cudaMemcpy(d_seg_table_temp,d_seg_table,512*424*sizeof(unsigned char),cudaMemcpyDeviceToDevice);
        peng_zhang<<< Grid, threadsPerBlock >>>(d_seg_table,d_seg_table_temp,512,424);
    }
    cudaDeviceSynchronize();
    label_initialize<<< Grid, threadsPerBlock >>>(d_seg_table,d_label,512,424);//
    int *array;
    cudaMalloc((void **)&array,6*sizeof(int));
    cudaMemset(array,0,6*sizeof(int));
    int *h_array;
    cudaHostAlloc((void **)&h_array,5*sizeof(int),cudaHostAllocDefault);
    int *d_flag;
    cudaMalloc((void **)&d_flag,1*sizeof(int));
    int h_flag=1;

    for (size_t nnn=0;nnn<3;nnn++)
    {
        eight_DLS_last<<< Grid, threadsPerBlock >>>(d_seg_table,d_label,array,d_flag,nnn);
    }
    cudaDeviceSynchronize();
    cudaMemcpy(h_array,array,5*sizeof(int),cudaMemcpyDeviceToHost);//如果change=0,说明没有任何改变，则退出循环
    int *h_center;
    cudaHostAlloc((void **)&h_center,3*sizeof(int),cudaHostAllocDefault);
    int *center;
    cudaMalloc((void **)&center,3*sizeof(int));
    int nu=0;

    for (int h1=0;h1<5;h1++)
    {
        if (h_array[h1]>0)
        {
            cudaMemset(center,0,3*sizeof(int));
            count<<< Grid, threadsPerBlock >>>(d_label,h_array[h1],center,512,424);
            cudaDeviceSynchronize();
            cudaMemcpy(h_center,center,3*sizeof(int),cudaMemcpyDeviceToHost);
            if (h_center[0]<20)//如果像素点很少的话忽略这个区域
            {
                h_array[h1]=0;
            }
            else
            {              
                cudaMemcpy(h_center,center,2*sizeof(int),cudaMemcpyDeviceToHost);
                ball_position[3*nu+0]=h_center[1]/(h_center[0]);
                ball_position[3*nu+1]=h_center[2]/(h_center[0]);
                ball_position[3*nu+2]=h_center[0];
                nu=nu+1;
            }
        }

    }
    cudaDeviceSynchronize();
    cudaFree(d_flag);
    cudaFreeHost(h_array);
    cudaFree(center);
    cudaFreeHost(h_center);
    cudaFree(array);
    cudaFree(d_seg_table_temp);
    return (nu);
}

__global__ void
projection_kernel(float *d_cloud, float *coeffi, int *histo_x, int *histo_y, int *histo_z,unsigned char *d_table)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 512;//0--512*424-1=217087
    __shared__ float coe[12];
    if (threadIdx.x<12)
        coe[threadIdx.x]=coeffi[threadIdx.x];

    if (idex<512*424)
    {
//        if ((d_cloud[idex*8+0]>-6000)&&(d_cloud[idex*8+0]<6000)&&(d_cloud[idex*8+1]>-4000)&&(d_cloud[idex*8+1]<2000)&&(d_cloud[idex*8+2]>500)&&(d_cloud[idex*8+2]<9000))//排除无效点干扰
//        {
            u_char* rgba = (u_char*) ( d_cloud + idex*8+4 );
             //颜色分割
             unsigned char re;
             re=d_table[rgba[0]/4*64*64 + rgba[1]/4*64 + rgba[2]/4];
             if(re!=0)//黄色像素
             {

//                 float xj=d_cloud[idex*8+0];
//                 float yj=d_cloud[idex*8+1];
//                 float zj=d_cloud[idex*8+2];//kinect坐标系
//                 double distance_to_plane = coeffi[0]*xj + (coeffi[1])*yj + (coeffi[2])*zj + coeffi[3]*1000;
                 float xr= coe[0]*d_cloud[idex*8+0]+coe[1]*d_cloud[idex*8+1]+coe[2]*d_cloud[idex*8+2]+coe[3]*1000;//直接转换到机器人坐标系
                 float yr= coe[4]*d_cloud[idex*8+0]+coe[5]*d_cloud[idex*8+1]+coe[6]*d_cloud[idex*8+2]+coe[7]*1000;
                 float zr= coe[8]*d_cloud[idex*8+0]+coe[9]*d_cloud[idex*8+1]+coe[10]*d_cloud[idex*8+2]+coe[11]*1000;
                 //因为相机是向下倾斜的，为了垂直投影到地平面，X方向没差别，Z需要由相机的光轴方向变换到水平方向
                 int x = (int)((yr+10000)/50);
                 int z = (int)((xr)/50);
                 int w=z*400+x;
                 if ((zr>100)&&(zr<300))//&&(distance_to_plane<1)
                 {
                     atomicAdd(&(histo_z[w]),1);
                 }
                 if ((zr>240)&&(zr<860))//&&(distance_to_plane<1)
                 {
                     atomicAdd(&(histo_x[w]),1);
     //                printf("%f %f ",zj_,xr);
                 }
                 if ((zr>1900)&&(zr<2500))//&&(distance_to_plane<1)
                 {
                     atomicAdd(&(histo_y[w]),1);
                 }
             }

//        }

    }
}

//__device__ void kernel_up(int &loc, float *temp_pj)
//{

//}

//__device__ int *location;

__global__ void object_label(unsigned char *pj, int *histo_x, int *histo_y, int *histo_z)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 400;//0--512*424-1=217087
    int histx=histo_x[idex];
    int histy=histo_y[idex];
    int histz=histo_z[idex];
    if (idex<400*240)
    {
//        int z_thresh=
        if (idex<16000)//0-2m
        {
            pj[idex]=(histz>90)&&(histx>10)&&(histy<1)? (histx+histz):0;
        }
        if ((idex>16000)&&(idex<24000))//2-3m
        {
            pj[idex]=(histz>60)&&(histx>2)&&(histy<1)? (histx+histz):0;
        }
        if ((idex<28000)&&(idex>24000))//3.0-3.5m
        {
            pj[idex]=(histz>30)&&(histx>2)&&(histy<1)? (histx+histz):0;
        }
        if ((idex<32000)&&(idex>28000))//3.5-4m
        {
            pj[idex]=(histz>20)&&(histx>1)&&(histy<1)? (histx+histz):0;
        }
        if ((idex<40000)&&(idex>32000))//4-5m
        {
            pj[idex]=(histz>5)&&(histx<50)&&(histy<1)? (histx+histz):0;
        }
        if ((idex<44000)&&(idex>40000))//5-5.5m
        {
            pj[idex]=(histz>3)&&(histx<30)&&(histy<1)? (histx+histz):0;
        }
        if ((idex<56000)&&(idex>44000))//5.5-7m
        {
            pj[idex]=(histz>4)&&(histx<20)&&(histy<1)? (histx+histz):0;
        }
//        if((idex>=56000)&&(idex<60000))
//        {
//          pj[idex]=0;
//        }
//        if ((histo_z[idex]>2)&&(histo_x[idex]>0)&&(histo_y[idex]<2))//之前判断z大于多少的貌似也可以
//        {
//            pj[idex]=histo_x[idex]+histo_z[idex]+histo_y[idex];
//        }
//        else
//        {
//            pj[idex]=0;
//        }
//        pj[idex]=(histo_z[idex]>2)&&(histo_x[idex]>0)&&(histo_y[idex]<10)? (histo_x[idex]+histo_z[idex]+histo_y[idex]) : 0;

    }
}
__global__ void Gaussian1D_kernel(unsigned char *pj, double *kernel, size_t size, unsigned char *result)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 400;//0--512*424-1=217087
    if (idex<400*240)
    {
        int kCenter = floor(size/2.0);
        int nn;
//y  先处理

        for (int n = 0; n < size; n++)
        {
            nn = size - 1 - n;
            int posx = i + (n - kCenter);
            int posy = j;
            int w=posx*400+posy;
            if(posx >= 0 && posx < 400)
            {
                result[idex] += pj[w]*kernel[nn];
            }
        }


    }
}
__global__ void Gaussian1D_kernel_(unsigned char *result, double *kernel, size_t size, unsigned char *pj, int *d_label)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 400;//0--512*424-1=217087
    if (idex<400*240)
    {
        int kCenter = floor(size/2.0);
        int nn;
        if ( (j>9) && (j<230))
        {
                for (int n = 0; n < size; n++)
                {
                    nn = size - 1 - n;
                    int posx = j;
                    int posy = i + (n - kCenter);
                    int w=posy*400+posx;
                    if(posy >= 0 && posy < 240)
                    {
                        pj[idex] += result[w]*kernel[nn];
                    }
                }
//x//后处理

                if ((pj[idex]>0)&&(i>0))
                {
                    d_label[idex]=i;
    //                atomicAdd(&(ch1[0]),1);
                }
        }

    }
}
__global__ void compute_array(int *label,  int *array1)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 400;//0--512*424-1=217087
    if (idex<400*240)
    {
        if (label[idex]>0)
        {

                if ((label[idex]!=array1[0])&&(label[idex]!=array1[1])&&(label[idex]!=array1[2])&&(label[idex]!=array1[3])&&(label[idex]!=array1[4])&&(label[idex]!=array1[5])&&(label[idex]!=array1[6])&&(label[idex]!=array1[7])&&(label[idex]!=array1[8])&&(label[idex]!=array1[9]))
                {
                    for (int k=0;k<10;k++)
                    {
                        if (array1[k]==0)
                        {
                            atomicExch(&(array1[k]),label[idex]);
                            break;
                        }
                     }
               }
        }
    }
}
__global__ void computer_center(int *d_label,int *index,int *t_index,int *amount,int *i_amount,int *j_amount)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 400;//0--512*424-1=217087
    const int sidex =  threadIdx.y;
   __shared__ int region_index[1];
   __shared__ int pixel_amount[1];
   __shared__ int x_amount[1];
   __shared__ int y_amount[1];
   __shared__ unsigned char s_label[192];
   __shared__ int flag[1];
   region_index[0]=0;
   pixel_amount[0]=0;
   x_amount[0]=0;
   y_amount[0]=0;
   flag[0]=0;
   if (sidex<192)
   {
       s_label[sidex]=d_label[idex];
   __syncthreads();
   if (s_label[sidex]>0)
   {
       region_index[0]=s_label[sidex];
       atomicAdd(&(pixel_amount[0]),1);
       atomicAdd(&(x_amount[0]),i);
       atomicAdd(&(y_amount[0]),j);
       flag[0]=1;
   }
   }
   __syncthreads();

   index[blockIdx.x]=flag[0];//0 1 0 1 1 0 0 0 1 1 1为了方便进行求前缀和
   t_index[blockIdx.x]=region_index[0];
   amount[blockIdx.x]=pixel_amount[0];
   i_amount[blockIdx.x]=x_amount[0];
   j_amount[blockIdx.x]=y_amount[0];
   __syncthreads();

}
__global__ void computer_center1(int *output,int *index,int *t_index,int *amount,int *i_amount,int *j_amount,int *output_amount,int *output_x,int *output_y)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;//blockIdx.x=1,blockIdx.y=424,blockDim.x=512,blockDim.y=1
    int j = threadIdx.y + blockIdx.y * blockDim.y;//threadIdx.x=512,threadIdx.y=1
    const int idex = (i) + (j) * 400;//0--512*424-1=217087
    const int sidex =  threadIdx.y;
   int space=1;
   //可能可以用动态并行做
   if(blockIdx.x==0)
   {
       int tt=0;
//       printf("%d ",index[sidex]);//到这儿的时候并不是所有块都完成了前面的index赋值，所以得到的数据不完整
       for (int d=1;d<=400;d=d*2)
       {
           int temp=index[sidex];
           int neighbor=0;
           if ((sidex-space>0))
               neighbor=index[sidex-space];
           __syncthreads();
           if (sidex<space)
               continue;
           else
               tt=temp+neighbor;//index的改变导致neighbor读取的错误
           space=space*2;
           __syncthreads();
           index[sidex]=tt;
       }
   }
   __syncthreads();
//   //index的最后一个数是代表有多少个列有目标值
   if (blockIdx.x==0)
   {

//       int current=sidex;
//       int later=current+1;
       //index这一步没问题。得到的index[399]个数也是对的，下面的判断条件有误
       if (sidex<399)//only use the even(偶数) threads, and the position in index is odd（奇数）.
       {
           if (index[sidex+1]==(index[sidex]+1))
           {
//               printf("even %d %d \n",index[sidex],index[sidex+1]);
               output[index[sidex]]=t_index[sidex+1];
               output_amount[index[sidex]]=amount[sidex+1];
               output_x[index[sidex]]=i_amount[sidex+1];
               output_y[index[sidex]]=j_amount[sidex+1];

           }
       }
   }
   __syncthreads();
   output[99]=index[399];//index最后一个数放的是最终有数值的块的个数
}
int project2D(float *d_cloud,  unsigned char *pj, float *aa,float *obstacle_position,unsigned char *d_table,int *histo_x,int *histo_y,int *histo_z,double *d_kernel,size_t kernel_size,unsigned char *result_, int *label, int *h_label)
{
    int *d_index,*d_t_index,*d_amount,*dx_amount,*dy_amount,*d_output;
    int *output_amount,*output_x, *output_y;
    cudaMalloc((void **)&d_index,400*sizeof(int));
    cudaMalloc((void **)&d_t_index,400*sizeof(int));
    cudaMalloc((void **)&d_amount,400*sizeof(int));
    cudaMalloc((void **)&dx_amount,400*sizeof(int));
    cudaMalloc((void **)&dy_amount,400*sizeof(int));
    cudaMalloc((void **)&d_output,100*sizeof(int));
    cudaMalloc((void **)&output_amount,100*sizeof(int));
    cudaMalloc((void **)&output_x,100*sizeof(int));
    cudaMalloc((void **)&output_y,100*sizeof(int));

    cudaMemset(histo_x,0,400*240*sizeof(int));
    cudaMemset(histo_y,0,400*240*sizeof(int));
    cudaMemset(histo_z,0,400*240*sizeof(int));
    cudaMemset(result_,0,400*240*sizeof(unsigned char));
    cudaMemset(label,0,400*240*sizeof(int));
    cudaMemset(h_label,0,400*240*sizeof(int));

    cudaMemset(d_amount,0,400*sizeof(int));
    cudaMemset(dx_amount,0,400*sizeof(int));
    cudaMemset(dy_amount,0,400*sizeof(int));
    cudaMemset(d_index,0,400*sizeof(int));
    cudaMemset(d_t_index,0,400*sizeof(int));
    cudaMemset(d_output,0,100*sizeof(int));
    cudaMemset(output_amount,0,100*sizeof(int));
    cudaMemset(output_x,0,100*sizeof(int));
    cudaMemset(output_y,0,100*sizeof(int));

    projection_kernel<<< Grid, threadsPerBlock >>>(d_cloud,aa,histo_x,histo_y,histo_z,d_table);
    cudaDeviceSynchronize();
    object_label<<< Grid_, threadsPerBlock >>>(pj, histo_x, histo_y, histo_z);
    cudaDeviceSynchronize();
    Gaussian1D_kernel<<< Grid_, threadsPerBlock >>>(pj, d_kernel, kernel_size, result_);

    Gaussian1D_kernel_<<< Grid_, threadsPerBlock >>>(result_, d_kernel, kernel_size, pj, label);
    cudaDeviceSynchronize();

    for (int nh=0;nh<3;nh++)
    {
        eight_DLS_last_obstacle<<< Grid_, threadsPerBlock >>>(pj,label,400,240);
    }
    cudaDeviceSynchronize();
    int *output=new int[100];
    int *amount=new int[100];
    int *x_amount=new int[100];
    int *y_amount=new int[100];

    computer_center<<< Grid1, threadsPerBlock1 >>>(label,d_index,d_t_index,d_amount,dx_amount,dy_amount);
    cudaDeviceSynchronize();

    computer_center1<<< Grid1, threadsPerBlock1 >>>(d_output,d_index,d_t_index,d_amount,dx_amount,dy_amount,output_amount,output_x,output_y);
    cudaDeviceSynchronize();

    cudaMemcpy(output,d_output,100*sizeof(int),cudaMemcpyDeviceToHost);
    cudaMemcpy(amount,output_amount,100*sizeof(int),cudaMemcpyDeviceToHost);
    cudaMemcpy(x_amount,output_x,100*sizeof(int),cudaMemcpyDeviceToHost);
    cudaMemcpy(y_amount,output_y,100*sizeof(int),cudaMemcpyDeviceToHost);
    int nu=0;
    vector<int> center_index, center_amount, center_x, center_y;
    if (output[99]>0)
    {
    center_index.push_back(output[0]);
    center_amount.push_back(amount[0]);
    center_x.push_back(x_amount[0]);
    center_y.push_back(y_amount[0]);

    for (int g=1;g<output[99];g++)
    {
        int numb=0;
        for (int l=0;l<center_index.size();l++)
        {
            if (output[g]==center_index.at(l))
            {
                center_amount.at(l)+=amount[g];
                center_x.at(l)+=x_amount[g];
                center_y.at(l)+=y_amount[g];
                break;
            }
            else
            {
                numb++;
                continue;
            }
        }
        if (numb==center_index.size())
        {
            center_index.push_back(output[g]);
            center_amount.push_back(amount[g]);
            center_x.push_back(x_amount[g]);
            center_y.push_back(y_amount[g]);
        }
    }

    for (int hq=0;hq<center_index.size();hq++)
    {
        if (center_amount.at(hq)==0)
            continue;
        float x = center_x.at(hq)/(center_amount.at(hq))*50+(-10000);
        float y = center_y.at(hq)/(center_amount.at(hq))*50;
        float value=-0.02181*y+178.7;
        if (center_amount.at(hq)>4 && center_amount.at(hq)>0.25*value)
        {
           obstacle_position[2*nu+0]=x;
           obstacle_position[2*nu+1]=y;
           nu++;
        }
    }
    }
    cudaFree(d_index);
    cudaFree(d_t_index);
    cudaFree(d_amount);
    cudaFree(dx_amount);
    cudaFree(dy_amount);
    cudaFree(d_output);
    cudaFree(output_amount);
    cudaFree(output_x);
    cudaFree(output_y);

    free(output);
    free(amount);
    free(x_amount);
    free(y_amount);
    return (nu);

}
