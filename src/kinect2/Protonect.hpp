#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string.h> // memcpy
using namespace std;
/// [headers]
#include <libfreenect2/libfreenect2.hpp>     ///find open control device, get ir color camera parameters
#include <libfreenect2/frame_listener_impl.h> ///wait for new frame
#include <libfreenect2/registration.h>   /// Register depth to color, create point clouds.
#include <libfreenect2/packet_pipeline.h>///Implement various methods to decode color and depth images with different performance and platform support
#include <libfreenect2/logger.h>
/// [headers]
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
#include "viewer.h"
#include <pcl/visualization/cloud_viewer.h>
#endif
#include <time.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#include "produce_pc_kernel.h"
#include "texture_types.h"
//texture<float, cudaTextureType1D, cudaReadModeElementType> texref;
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

class Protonect
{
public:
     void initcuda(libfreenect2::Freenect2Device *dev, libfreenect2::Registration* registration);

};
void Protonect::initcuda(libfreenect2::Freenect2Device *dev, libfreenect2::Registration* registration)
{

         cudaError_t err = cudaSuccess;
         const size_t imageSize = 4 * 424 * 512;//868352  (undistorted.bytes_per_pixel)*424*512
         const size_t rgbsize = 4 * 1920 * 1080;
         /// Init GPU variables and memery
         float* d_dis_depth, *d_map_x, *d_cloud, *d_c;
         int* d_map_dist, *d_map_yi;
         unsigned int* d_raw_rgb, *d_register;
         cudaMalloc((void**)&d_cloud,       imageSize*8);
         cudaMalloc((void**)&d_dis_depth,   imageSize);
         cudaMalloc((void**)&d_raw_rgb,     rgbsize  );
         cudaMalloc((void**)&d_map_dist,    imageSize);
         cudaMalloc((void**)&d_map_x,       imageSize);
         cudaMalloc((void**)&d_map_yi,      imageSize);
         cudaMalloc((void**)&d_c,           4*sizeof(float));
         const float cx(dev->getIrCameraParams().cx-0.5f), cy(dev->getIrCameraParams().cy-0.5f);
         const float fx(1/dev->getIrCameraParams().fx), fy(1/dev->getIrCameraParams().fy);
         const float para[] = {cx,cy,fx,fy};
         cudaMemcpy(d_map_dist,  registration->impl_->distort_map,          imageSize,       cudaMemcpyHostToDevice);
         cudaMemcpy(d_map_x,     registration->impl_->depth_to_color_map_x, imageSize,       cudaMemcpyHostToDevice);
         cudaMemcpy(d_map_yi,    registration->impl_->depth_to_color_map_yi,imageSize,       cudaMemcpyHostToDevice);//512*424
         cudaMemcpy(d_c,         &para,                                     4*sizeof(float), cudaMemcpyHostToDevice);
         unsigned char *h_register;
         cudaSetDeviceFlags(cudaDeviceMapHost);
         err = cudaHostAlloc((void**)&h_register, imageSize, cudaHostAllocMapped);
         if (err != cudaSuccess)
         {
             fprintf(stderr, "Failed to get device pointer register(error code %s)!\n", cudaGetErrorString(err));
             exit(EXIT_FAILURE);
         }
         libfreenect2::Frame registered(512, 424, 4, h_register);
         err = cudaHostGetDevicePointer((void**)&d_register, (void*)h_register, 0);
         if (err != cudaSuccess)
         {
             fprintf(stderr, "Failed to get device pointer pointer(error code %s)!\n", cudaGetErrorString(err));
             exit(EXIT_FAILURE);
         }


}
