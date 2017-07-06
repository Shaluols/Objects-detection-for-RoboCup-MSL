/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file Protonect.cpp Main application file. */

#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <stdio.h>
#include <string.h> // memcpy
/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
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
#include <thrust/device_vector.h>
#include <thrust/reduce.h>

using namespace std;
bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;

//Doing non-trivial things in signal handler is bad. If you want to pause,
//do it in another thread.
//Though libusb operations are generally thread safe, I cannot guarantee
//everything above is thread safe when calling start()/stop() while
//waitForNewFrame().
void sigusr1_handler(int s)
{
  if (devtopause == 0)
    return;
/// [pause]
  if (protonect_paused)
    devtopause->start();
  else
    devtopause->stop();
  protonect_paused = !protonect_paused;
/// [pause]
}

//The following demostrates how to create a custom logger
/// [logger]
#include <fstream>
#include <cstdlib>
class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
  {
    if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
  bool good()
  {
    return logfile_.is_open() && logfile_.good();
  }
  virtual void log(Level level, const std::string &message)
  {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};
/// [logger]

/// [main]
/**
 * Main application entry point.
 *
 * Accepted argumemnts:
 * - cpu Perform depth processing with the CPU.
 * - gl  Perform depth processing with OpenGL.
 * - cl  Perform depth processing with OpenCL.
 * - <number> Serial number of the device to open.
 * - -noviewer Disable viewer window.
 */
int main(int argc, char *argv[])
/// [main]
{
  std::string program_path(argv[0]);
  std::cerr << "Version: " << LIBFREENECT2_VERSION << std::endl;
  std::cerr << "Environment variables: LOGFILE=<protonect.log>" << std::endl;
  std::cerr << "Usage: " << program_path << " [-gpu=<id>] [gl | cl | clkde | cuda | cudakde | cpu] [<device serial>]" << std::endl;
  std::cerr << "        [-noviewer] [-norgb | -nodepth] [-help] [-version]" << std::endl;
  std::cerr << "        [-frames <number of frames to process>]" << std::endl;
  std::cerr << "To pause and unpause: pkill -USR1 Protonect" << std::endl;
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";
  ros::Time::init();
  ros::init(argc, argv, "kinect2_driver");//node name
  ros::NodeHandle nh("~");
  std::string device_id_;
  device_id_ = " ";
  nh.getParam("device_id",device_id_);
  ros::Publisher pc_pub;
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_original", 1);

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
  // avoid flooing the very slow Windows console with debug messages
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
#else
  // create a console logger with debug level (default is console logger with info level)
/// [logging]
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
/// [logging]
#endif
/// [file logging]
  MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
  if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);
  else
    delete filelogger;
/// [file logging]

/// [context]
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
/// [context]

  std::string serial = "";

  bool viewer_enabled = false;
  bool enable_rgb = true;
  bool enable_depth = true;
  int deviceId = -1;
  size_t framemax = -1;

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg(argv[argI]);

    if(arg == "-help" || arg == "--help" || arg == "-h" || arg == "-v" || arg == "--version" || arg == "-version")
    {
      // Just let the initial lines display at the beginning of main
      return 0;
    }
    else if(arg.find("-gpu=") == 0)
    {
      if (pipeline)
      {
        std::cerr << "-gpu must be specified before pipeline argument" << std::endl;
        return -1;
      }
      deviceId = atoi(argv[argI] + 5);
    }
    else if(arg == "cpu")
    {
      if(!pipeline)
/// [pipeline]
        pipeline = new libfreenect2::CpuPacketPipeline();
/// [pipeline]
    }
    else if(arg == "gl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "clkde")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLKdePacketPipeline(deviceId);
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cuda")
    {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
#else
      std::cout << "CUDA pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cudakde")
    {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::CudaKdePacketPipeline(deviceId);
#else
      std::cout << "CUDA pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
    {
      serial = arg;
    }
    else if(arg == "-noviewer" || arg == "--noviewer")
    {
      viewer_enabled = false;
    }
    else if(arg == "-norgb" || arg == "--norgb")
    {
      enable_rgb = false;
    }
    else if(arg == "-nodepth" || arg == "--nodepth")
    {
      enable_depth = false;
    }
    else if(arg == "-frames")
    {
      ++argI;
      framemax = strtol(argv[argI], NULL, 0);
      if (framemax == 0) {
        std::cerr << "invalid frame count '" << argv[argI] << "'" << std::endl;
        return -1;
      }
    }
    else
    {
      std::cout << "Unknown argument: " << arg << std::endl;
    }
  }

//  pipeline = new libfreenect2::CudaKdePacketPipeline(deviceId);
  pipeline = new libfreenect2::CudaPacketPipeline(deviceId);

  if (!enable_rgb && !enable_depth)
  {
    std::cerr << "Disabling both streams is not allowed!" << std::endl;
    return -1;
  }

/// [discovery]
  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }

  if (serial == "")
  {
    serial = freenect2.getDefaultDeviceSerialNumber();
  }
/// [discovery]

  if(pipeline)
  {
/// [open]
    dev = freenect2.openDevice(serial, pipeline);
    //----------------------------------------------------
    libfreenect2::Freenect2Device::Config config;
    config.EnableBilateralFilter = true;
    config.EnableEdgeAwareFilter = true;
    config.MinDepth = 0.1;
    config.MaxDepth = 12;
    dev->setConfiguration(config);
    /// [open]
/// [open]
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  devtopause = dev;

  signal(SIGINT,sigint_handler);
#ifdef SIGUSR1
  signal(SIGUSR1, sigusr1_handler);
#endif
  protonect_shutdown = false;

/// [listeners]
  int types = 0;
  if (enable_rgb)
    types |= libfreenect2::Frame::Color;
  if (enable_depth)
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
/// [listeners]

/// [start]
  if (enable_rgb && enable_depth)
  {
    if (!dev->start())
      return -1;
  }
  else
  {
    if (!dev->startStreams(enable_rgb, enable_depth))
      return -1;
  }

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
/// [start]
  unsigned char *h_register1;
/// [registration setup]
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registerer(512, 424, 4, h_register1);
/// [registration setup]

  size_t framecount = 0;
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
  Viewer viewer;
  if (viewer_enabled)
    viewer.initialize();
#else
  viewer_enabled = false;
#endif
  float time1 = (float)clock(); //开始时间
 pcl::visualization::PCLVisualizer::Ptr visualizer;
 pcl::PointCloud<pcl::PointXYZRGB> cloud(512,424);
pcl::PointCloud<pcl::PointXYZRGB> cloud_(512,424);
 pcl::PCLPointCloud2 pcl_pc;
 sensor_msgs::PointCloud2 ros_pc;

// cudaError_t err = cudaSuccess;
// const size_t imageSize = 4 * 424 * 512;//868352  (undistorted.bytes_per_pixel)*424*512
// const size_t rgbsize = 4 * 1920 * 1080;
// /// Init GPU variables and memery
// float* d_dis_depth, *d_map_x, *d_cloud, *d_c;
// int* d_map_dist, *d_map_yi;
// unsigned int* d_raw_rgb, *d_register;
// //--------------------------------------------------------------------------------------------------------------
// float *filter_map = NULL;//1920×1080+1920×2
// float *d_filter_map, *d_p_filter_map;
// float *p_filter_map = NULL;//每一次加一行
// const int size_filter_map = 1920 * 1080+1920*1*2;
// filter_map = new float[size_filter_map];
// //    p_filter_map = filter_map + 1920*1;
// for(float *it = filter_map, *end = filter_map + size_filter_map; it != end; ++it){
//   *it = std::numeric_limits<float>::infinity();
// }
// cudaMalloc((void**)&d_filter_map,  size_filter_map*4);
// //    cudaMalloc((void**)&d_p_filter_map, (size_filter_map+1920)*4);
// cudaMemcpy(d_filter_map, filter_map,          size_filter_map*4,       cudaMemcpyHostToDevice);

// //-------------------------------------------------------------------------------------------------------------
// cudaMalloc((void**)&d_cloud,       imageSize*8);
// cudaMalloc((void**)&d_dis_depth,   imageSize);
// cudaMalloc((void**)&d_raw_rgb,     rgbsize  );
// cudaMalloc((void**)&d_map_dist,    imageSize);
// cudaMalloc((void**)&d_map_x,       imageSize);
// cudaMalloc((void**)&d_map_yi,      imageSize);
// cudaMalloc((void**)&d_c,           5*sizeof(float));
// const float cx(dev->getIrCameraParams().cx-0.5f), cy(dev->getIrCameraParams().cy-0.5f);
// const float fx(1/dev->getIrCameraParams().fx), fy(1/dev->getIrCameraParams().fy);
// const float bad_point = std::numeric_limits<float>::quiet_NaN();
// const float para[] = {cx,cy,fx,fy, bad_point};
// cudaMemcpy(d_map_dist,  registration->impl_->distort_map,          imageSize,       cudaMemcpyHostToDevice);
// cudaMemcpy(d_map_x,     registration->impl_->depth_to_color_map_x, imageSize,       cudaMemcpyHostToDevice);
// cudaMemcpy(d_map_yi,    registration->impl_->depth_to_color_map_yi,imageSize,       cudaMemcpyHostToDevice);//512*424
// cudaMemcpy(d_c,         &para,                                     5*sizeof(float), cudaMemcpyHostToDevice);
// unsigned char *h_register;
// cudaSetDeviceFlags(cudaDeviceMapHost);
// err = cudaHostAlloc((void**)&h_register, imageSize, cudaHostAllocMapped);
// if (err != cudaSuccess)
// {
//     fprintf(stderr, "Failed to get device pointer register(error code %s)!\n", cudaGetErrorString(err));
//     exit(EXIT_FAILURE);
// }
// libfreenect2::Frame registered(512, 424, 4, h_register);//第四个参数是数据
// err = cudaHostGetDevicePointer((void**)&d_register, (void*)h_register, 0);
// if (err != cudaSuccess)
// {
//     fprintf(stderr, "Failed to get device pointer pointer(error code %s)!\n", cudaGetErrorString(err));
//     exit(EXIT_FAILURE);
// }
 float time2 = (float)clock(); //结束时间
 printf("prepare time: %.2fms\t\n",(time1-time2)*1000 / CLOCKS_PER_SEC); //输出
//    /// [loop start]
/// [loop start]
  while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
  {
    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
      std::cout << "timeout!" << std::endl;
      return -1;
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
/// [loop start]
//    cudaMemcpy(d_dis_depth, depth->data, imageSize, cudaMemcpyHostToDevice);
//    cudaMemcpy(d_raw_rgb,   rgb->data,   rgbsize,   cudaMemcpyHostToDevice);
    float x,y,z,rgb_;
    if (enable_rgb && enable_depth)
    {
/// [registration]
        registration->apply(rgb, depth, &undistorted, &registerer);
        for (int i=0; i<424; i++)
            for (int j=0; j<512; j++)
            {
                int index=i*512+j;
                registration->getPointXYZRGB(&undistorted,&registerer,i,j,x,y,z,rgb_);
                cloud_.points[index].x=x;
                cloud_.points[index].y=y;
                cloud_.points[index].z=z;
                cloud_.points[index].rgb=rgb_;
            }

//        float time_start = (float)clock(); //开始时间
//        registration_kernel(d_dis_depth, d_raw_rgb, d_register, d_map_dist, d_map_x, d_map_yi, d_c, d_cloud, d_filter_map);

//        float time_now = (float)clock(); //结束时间
//        printf("registration time: %.2fms\t\n",(time_now-time_start)*1000 / CLOCKS_PER_SEC); //输出
//        float time_last = time_now;
//        cudaMemcpy(cloud.points.data(), d_cloud, imageSize*8, cudaMemcpyDefault);
//        time_now = (float)clock(); //结束时间
//        printf("copy time: %.2fms\t\n",(time_now-time_last)*1000 / CLOCKS_PER_SEC); //输出
        if(1)
        {
            if( !visualizer )
            {
                visualizer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer("Cloud Viewer"));
                visualizer->setCameraPosition(0, 0, -4, 0, 0, 1,0, -1, 0);
                visualizer->addPointCloud(cloud_.makeShared(),"cloud");
                visualizer->spinOnce();
            }
            else if( !visualizer->wasStopped() )
            {
                visualizer->updatePointCloud(cloud_.makeShared(),"cloud");
                visualizer->spinOnce();
            }
            else
            {
                visualizer->removeAllPointClouds();
                visualizer->close();
                visualizer.reset();
                break;
            }
        }
//        time_now = (double)clock(); //结束时间
//        printf("viewer time: %.2fms\t\n",(time_now-time_last)*1000 / CLOCKS_PER_SEC); //输出
//        time_last = time_now;
        pcl::toROSMsg(cloud_,ros_pc);
        ros_pc.header.stamp = ros::Time::now();
        ros_pc.header.frame_id = device_id_;
        pc_pub.publish(ros_pc);
/// [registration]
    }

    framecount++;
    if (!viewer_enabled)
    {
      if (framecount % 100 == 0)
        std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
      listener.release(frames);
      continue;
    }

#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
//    if (enable_rgb)
//    {
//      viewer.addFrame("RGB", rgb);
//    }
//    if (enable_depth)
//    {
//      viewer.addFrame("ir", ir);
//      viewer.addFrame("depth", depth);
//    }
//    if (enable_rgb && enable_depth)
//    {
//      viewer.addFrame("registered", &registered);
//    }

    protonect_shutdown = protonect_shutdown || viewer.render();
#endif

/// [loop end]
    listener.release(frames);
    /** libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100)); */
  }
/// [loop end]
//  cudaFree(d_cloud);
//  cudaFree(d_c);
//  cudaFree(d_raw_rgb);
//  cudaFree(d_dis_depth);
//  cudaFree(d_map_dist);
//  cudaFree(d_map_x);
//  cudaFree(d_map_yi);
//  cudaFreeHost(h_register);
//  cudaFreeHost(d_filter_map);
  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
/// [stop]
  dev->stop();
  dev->close();
/// [stop]

  delete registration;

  return 0;
}
