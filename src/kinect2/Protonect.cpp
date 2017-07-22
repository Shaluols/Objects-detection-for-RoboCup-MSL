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

#include "Protonect.h"
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
Protonect::Protonect()
{

}
Protonect::~Protonect()
{

}

double Protonect::dtime()
{
    double tseconds=0.0;
    struct timeval mytime;
    gettimeofday(&mytime,(struct timezone *)0);
    tseconds=(double)(mytime.tv_sec+mytime.tv_usec*1.0e-6);
    return tseconds;
}

void Protonect::cicle()
{

    std::string device_id_;
    device_id_ = " ";
    nh.getParam("device_id",device_id_);
    obstacle_ball_pub = nh.advertise<nubot_common::object_info>("ball_obstacle_position", 1);



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
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
  /// [context]

    std::string serial = "";

    bool viewer_enabled = false;
    bool enable_rgb = true;
    bool enable_depth = true;
    int deviceId = -1;
    size_t framemax = -1;

    pipeline = new libfreenect2::CudaKdePacketPipeline(deviceId);
  //  pipeline = new libfreenect2::CudaPacketPipeline(deviceId);

    if (!enable_rgb && !enable_depth)
    {
      std::cerr << "Disabling both streams is not allowed!" << std::endl;

    }

  /// [discovery]
    if(freenect2.enumerateDevices() == 0)
    {
      std::cout << "no device connected!" << std::endl;

    }

    if (serial == "")
    {
      serial = freenect2.getDefaultDeviceSerialNumber();
    }
  /// [discovery]
      dev = freenect2.openDevice(serial, pipeline);
      //----------------------------------------------------
      libfreenect2::Freenect2Device::Config config;
      config.EnableBilateralFilter = true;
      config.EnableEdgeAwareFilter = true;
      config.MinDepth = 0.1;
      config.MaxDepth = 12;
      dev->setConfiguration(config);
    if(dev == 0)
    {
      std::cout << "failure opening device!" << std::endl;

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
          printf("dev didn't start!\n");

    }
    else
    {
      if (!dev->startStreams(enable_rgb, enable_depth))
          printf("dev didn't start!\n");

    }

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
  /// [start]
  /// [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  //  libfreenect2::Frame undistorted(512, 424, 4), registerer(512, 424, 4, h_register1);
  /// [registration setup]

    size_t framecount = 0;
  #ifdef EXAMPLES_WITH_OPENGL_SUPPORT
    Viewer viewer;
    if (viewer_enabled)
      viewer.initialize();
  #else
    viewer_enabled = false;
  #endif
    //----------------------define constant variables-----------------------------------
    //----------------------define constant variables-----------------------------------

  //----------------------define variables-----------------------------------
  //----------------------define variables-----------------------------------
    pcl::PointCloud<pcl::PointXYZRGBA> cloud(512,424);
    imageSize = 4 * 424 * 512;//868352  (undistorted.bytes_per_pixel)*424*512
    rgbsize = 4 * 1920 * 1080;
    /// Init GPU variables and memery




    cudaError_t err = cudaSuccess;
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
    libfreenect2::Frame registered(512, 424, 4, h_register);
  //--------------------------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------------------------

    if((environment = getenv("AGENT"))==NULL) //Return the value of envariable NAME
    {
        ROS_ERROR("this agent number is not read by robot");

    }
    agent = atoi(environment);//Convert a string to an integer
    ss<<agent;
               //获取home目录
    home=getenv("HOME");

    sh<<home;
    calibration_path=std::string(sh.str()+"/libfreenect2-master/src/kinect2/calib_file/");
    color_table_path = calibration_path + "/CTableBGR.dat";
    tf_matrix_path   = calibration_path + "/tf_matrix.txt";
    color_segmenter_ = new nubot::ColorSegment(color_table_path.c_str());
    float aa[12]={0};
  //  FILE *fpRead=fopen("/home/ubuntu/libfreenect2-master/src/kinect2/calib_file/coefficients_plane.txt","r");
    FILE *fpRead_tf=fopen("/home/ubuntu/libfreenect2-master/src/kinect2/calib_file/tf_matrix.txt","r");
    if (fpRead_tf==NULL)
        printf("can't read the tf file \n");
    for (int i=0;i<12;i++)
    {
        fscanf(fpRead_tf,"%f",&aa[i]);
    }

    grayBytes = 512*424;//原始图的大小
    table_size = 64*64*64*sizeof(unsigned char);//颜色分割表的大小

    h_num_yellow=new int[1];//计算 registration_kernel中黄色像素点的个数
    memset(h_num_yellow,0,sizeof(int));
    cudaMalloc((void**)&d_seg_table,grayBytes);//输入region_search的颜色分割图，黄色想色是0
    cudaMalloc((void**)&seg,grayBytes);
    cudaMalloc((void**)&d_label,512*424*sizeof(int));

    ball_position=new float[10];//前两个是中心的x,y值，后两个是宽和高的一半用于计算显示的rect起始点
    obstacle_position=new float[40];//前两个是中心的x,y值，后两个是宽和高的一半用于计算显示的rect起始点
    cudaMalloc((void**)&d_table,table_size);
    cudaMemcpy(d_table, color_segmenter_->table_, table_size, cudaMemcpyHostToDevice);

    cudaMalloc((void **)&pj, 400*240*sizeof(unsigned char));
    cudaMalloc((void **)&coeffi, 12*sizeof(float));
    cudaMemcpy(coeffi, &aa, 12*sizeof(float),cudaMemcpyHostToDevice);
    h_ppj=(unsigned char *)malloc(400*240*sizeof(unsigned char));

    cudaMalloc((void **)&histo_x,400*240*sizeof(int));
    cudaMalloc((void **)&histo_y,400*240*sizeof(int));
    cudaMalloc((void **)&histo_z,400*240*sizeof(int));

    kernel = LaterMethods::getGaussian1D(2,1);

    cudaMalloc((void **)&d_kernel,kernel.size()*sizeof(double));
    cudaMemcpy(d_kernel,&kernel[0],kernel.size()*sizeof(double),cudaMemcpyHostToDevice);
    kernel_size=kernel.size();

    cudaMalloc((void **)&result_, 400*240*sizeof(unsigned char));

    ttime=new float[7];
    ttime1=new float[6];
    cudaMalloc((void **)&label, 400*240*sizeof(int));

    cudaHostAlloc((void **)&h_label, 400*240*sizeof(int),cudaHostAllocDefault);
    memset(ball_position,0,15*sizeof(float));
    memset(obstacle_position,0,40*sizeof(float));

    ros::param::get("/kinect2_driver/color_calibrating",color_calibrating);

    ros::param::get("/kinect2_driver/cali_tf",cali_tf);

    ros::param::get("/kinect2_driver/view_color_seg",view_color_seg);

    ros::param::get("/kinect2_driver/point_cloud_view",point_cloud_view);

    ros::param::get("/kinect2_driver/ball_detection_view",ball_detection_view);

    ros::param::get("/kinect2_driver/obstacle_detection_view",obstacle_detection_view);
    #define KINECT_POS_X 0.12
    #define KINECT_POS_Y -0.07
    #define KINECT_POS_Z 0.49

  //  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer11 (new pcl::visualization::PCLVisualizer ("point cloud Viewer"));
  //  viewer11->setCameraPosition(0, 0, -4000, 0, 0, 1,0, -1, 0);
                  //用来记录时间的
    cppfile.open("/home/ubuntu/diatance_pixel.txt");
    cppfile<<"distance"<<"\t"<<"ball pixels number"<<"\t";
    cppfile<<"\n";
    ball_time.open("/home/ubuntu/ball_time.txt");
    ball_time<<"pengzhang_fushi"<<"\t"<<"initialize"<<"\t"<<"ccl"<<"\t"<<"center single"<<"\t"<<"center"<<"\t"<<"free"<<"\t";
    ball_time<<"\n";
    obs_time.open("/home/ubuntu/obs_time.txt");
    obs_time<<"cudamemset"<<"\t"<<"projct"<<"\t"<<"object_label"<<"\t"<<"Gaussian"<<"\t"<<"ccl"<<"\t"<<"center"<<"\t"<<"free"<<"\t";
    obs_time<<"\n";
    time1.open("/home/ubuntu/time.txt");
    time1<<"driver"<<"\t"<<"registration"<<"\t"<<"cloud copy"<<"\t"<<"obs"<<"\t"<<"ball single"<<"\t"<<"ball all"<<"\t"<<"total"<<"\t";
    time1<<"\n";

    while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
    {
        double clocktime0=dtime();

        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
        {
            std::cout << "timeout!" << std::endl;
            break ;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];//BGRA format
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        cudaStream_t *streams = (cudaStream_t *) malloc(2 * sizeof(cudaStream_t));
  //      double tstart=dtime();
         for (int i = 0; i < 2; i++)
         {
             cudaStreamCreate(&(streams[i]));
         }
        cudaMemcpyAsync(d_dis_depth, depth->data, imageSize, cudaMemcpyHostToDevice, streams[0]);//这个应该也是实时的
        cudaMemcpyAsync(d_raw_rgb,   rgb->data,   rgbsize,   cudaMemcpyHostToDevice, streams[1]);//每一帧过来都会处理
        for (int i = 0; i < 2; i++)
        {
            cudaStreamDestroy(streams[i]);
        }
         double tstart=dtime();
  //       printf("11 driver driver time = %f(ms)\n\n",(tstart-clocktime0)*1000);
         time1<<(tstart-clocktime0)*1000<<"\t";
        registration_kernel(d_dis_depth, d_raw_rgb, d_map_dist, d_map_x, d_map_yi, d_c, d_cloud, d_table, d_seg_table, h_num_yellow);
        cudaDeviceSynchronize();
        double tstart1=dtime();
  //      printf("22 registration time = %f(ms)\n\n",(tstart1-tstart)*1000);
        time1<<(tstart1-tstart)*1000<<"\t";
        cudaMemcpy(cloud.points.data(), d_cloud, imageSize*8, cudaMemcpyDeviceToHost);
        if (color_calibrating)
        {
            if (cloud.isOrganized())
            {
                rgb_image = cv::Mat(cloud.height, cloud.width, CV_8UC3);

                if (!cloud.empty())
                {
                    for (int h=0; h<rgb_image.rows; h++)
                    {
                        for (int w=0; w<rgb_image.cols; w++)
                        {
                            pcl::PointXYZRGBA point = cloud.at(w, h);
                            Eigen::Vector3i rgb = point.getRGBVector3i();
                            rgb_image.at<cv::Vec3b>(h,w)[0] = rgb[2];
                            rgb_image.at<cv::Vec3b>(h,w)[1] = rgb[1];
                            rgb_image.at<cv::Vec3b>(h,w)[2] = rgb[0];
                        }
                    }
                }
            }
            if(cv::imwrite((calibration_path+"rgb.bmp").c_str(),rgb_image));
            cout << "rgbImage Saved!" << endl;
            color_calibrating=false;
        }
        double tstart2=dtime();
  //      printf("33 cloud copy and color calibrating time = %f(ms)\n\n",(tstart2-tstart1)*1000);
        time1<<(tstart2-tstart1)*1000<<"\t";
        cudaMemset(pj,0,400*240*sizeof(unsigned char));
        num_obstacle_candidate = project2D(d_cloud, pj, coeffi, obstacle_position,d_table,histo_x,histo_y,histo_z,d_kernel,kernel_size,result_,label,h_label,ttime);
        //printf("!!!!!!!!!!!!!!number obs %d\n",num_obstacle_candidate);
        //        obs_time<<ttime[0]<<"\t"<<ttime[1]<<"\t"<<ttime[2]<<"\t"<<ttime[3]<<"\t"<<ttime[4]<<"\t"<<ttime[5]<<"\t"<<ttime[6]<<"\t";
//        obs_time<<"\n";
        cudaDeviceSynchronize();
        double tstart3=dtime();
  //      printf("44 obstacle detection time = %f(ms)\n\n",(tstart3-tstart2)*1000);
        time1<<(tstart3-tstart2)*1000<<"\t";
        nubot_common::object_info obstacle_ball_position;
        printf("yellow number %d \n",*h_num_yellow);
        if (num_obstacle_candidate==0)
           obstacle_ball_position.obs_know=false;
        if (*h_num_yellow<20)
         {
               printf("no ball!!!!!!!!!!!!!!!!!!!\n\n");
         }
        else
         {
            double tstart4=dtime();
            num_ball_candidate=CCL(d_seg_table,d_label,ball_position,ttime1);
          //  printf("no num_ball_candidate!%d\n\n",num_ball_candidate);
            cudaDeviceSynchronize();
            double tstart5=dtime();
            ball_time<<ttime1[0]<<"\t"<<ttime1[1]<<"\t"<<ttime1[2]<<"\t"<<ttime1[3]<<"\t"<<ttime1[4]<<"\t"<<ttime1[5]<<"\t";
            ball_time<<"\n";
  //          printf("551 ball ball time = %f(ms)\n\n",(tstart5-tstart4)*1000);
            time1<<(tstart5-tstart4)*1000<<"\t";
            if (num_ball_candidate==0)
               obstacle_ball_position.ball_know=false;

            for (int r=0;r<num_ball_candidate;r++)
            {
             int temp_x = (int)ball_position[r*3+0];
             int temp_y = (int)ball_position[r*3+1];
             if(      100000 != cloud.at(temp_x,   temp_y  ).z )
                   ball_point = cloud.at(temp_x,   temp_y  );
             else if( 100000 != cloud.at(std::max(temp_x-2000,0),                          std::max(temp_y-2000,0)).z )
                   ball_point = cloud.at(std::max(temp_x-2000,0),                          std::max(temp_y-2000,0));
             else if( 100000 != cloud.at(std::min(temp_x+2000,int(cloud.width)-1), std::max(temp_y-2000,0)).z )
                   ball_point = cloud.at(std::min(temp_x+2000,int(cloud.width)-1), std::max(temp_y-2000,0));
             else if( 100000 != cloud.at(std::max(temp_x-2000,0),                          std::min(temp_y+2000,int(cloud.height)-1)).z )
                   ball_point = cloud.at(std::max(temp_x-2000,0),                          std::min(temp_y+2000,int(cloud.height)-1));
             else if( 100000 != cloud.at(std::min(temp_x+2000,int(cloud.width)-1), std::min(temp_y+2000,int(cloud.height)-1)).z )
                   ball_point = cloud.at(std::min(temp_x+2000,int(cloud.width)-1), std::min(temp_y+2000,int(cloud.height)-1));

             if(ball_position[r*3+2]>(0.33*3670000000*pow(ball_point.z,-1.972)))
             {
             float xk=ball_point.x*aa[0]+ball_point.y*aa[1]+ball_point.z*aa[2]+aa[3]*1000;
             float yk=(ball_point.x*aa[4]+ball_point.y*aa[5]+ball_point.z*aa[6]+aa[7]*1000);
             float zk=ball_point.x*aa[8]+ball_point.y*aa[9]+ball_point.z*aa[10]+aa[11]*1000;
             printf("3D ball position left up foreward %f %f %f\n\n",ball_point.x,ball_point.y,ball_point.z);
             printf("robot 3D ball position  forward left up %f %f %f\n\n",xk,yk,zk);
  //           if (xk>1300 && zk < 1200 && obstacle_ball_position.ball_know)

             if (xk>0)
            {
             obstacle_ball_position.ball_know=true;
             geometry_msgs::Point ball_pp;
             ball_pp.x=xk;
             ball_pp.y=yk;
             ball_pp.z=zk;

             obstacle_ball_position.ball_pos.push_back(ball_pp);
            }

            }
            }
        }
        double clocktime1=dtime();
  //      printf("55 ball detection time = %f(ms)\n\n",(clocktime1-tstart3)*1000);
        time1<<(clocktime1-tstart3)*1000<<"\t"<<(clocktime1-clocktime0)*1000<<"\t";
        time1<<"\n";
        if (view_color_seg)
        {
            unsigned char *hpj=(unsigned char *)malloc(400*240*sizeof(unsigned char));;
            cudaMemcpy(hpj,pj,400*240*sizeof(unsigned char),cudaMemcpyDeviceToHost);

            unsigned char *h_ppjj=(unsigned char *)malloc(512*424*sizeof(unsigned char));
            cudaMemcpy(h_ppjj,d_seg_table,512*424*sizeof(unsigned char),cudaMemcpyDeviceToHost);
            cv::Mat hpj_imgg_(cv::Size(400, 240),CV_8UC1,hpj);
            cv::namedWindow("ball morphological operation");
            cv::imshow("obs morphological operation", hpj_imgg_*255);
            cv::Mat pj_imgg_(cv::Size(512, 424),CV_8UC1,h_ppjj);
            cv::namedWindow("ball morphological operation");
            cv::imshow("ball morphological operation", pj_imgg_*255);
            if (cloud.isOrganized())
            {
                rgb_image = cv::Mat(cloud.height, cloud.width, CV_8UC3);

                if (!cloud.empty())
                {
                    for (int h=0; h<rgb_image.rows; h++)
                    {
                        for (int w=0; w<rgb_image.cols; w++)
                        {
                            pcl::PointXYZRGBA point = cloud.at(w, h);
                            Eigen::Vector3i rgb = point.getRGBVector3i();
                            rgb_image.at<cv::Vec3b>(h,w)[0] = rgb[2];
                            rgb_image.at<cv::Vec3b>(h,w)[1] = rgb[1];
                            rgb_image.at<cv::Vec3b>(h,w)[2] = rgb[0];
                        }
                    }
                }
            }
            cv::imshow("rgb_image_",rgb_image);

            cv::waitKey(10);
        }
        if (cali_tf)
          {
              float temp=0.0;
              bool flag_temp=false;
              float tt=0.0;
              char key = '0';
              std::cout << ":make sure the point is ok and the ball is at facing direction of robot(angle=0)!" << std::endl;
              std::cout << ":are you satisfied with this result, and save it? [y/n]:" << std::endl;
              std::cin >> key;
              if( key == 'y' || key == 'Y' )
              {
                 geometry_msgs::Point ball_pp;
                 ball_pp.x=ball_point.x;
                 ball_pp.y=ball_point.y;
                 ball_pp.z=ball_point.z;
                 tf_ball.push_back(ball_pp);
                 if (tf_ball.size()==5)
                 {
                     cali_tf=false;
                     for(int i=0;i<(tf_ball.size()-1);i++)
                     {
                         for (int j=i+1; j<tf_ball.size();j++)
                         {
                             if (tf_ball.at(i).z<tf_ball.at(j).z)
                             {
                                 tt=(tf_ball.at(j).y-tf_ball.at(i).y)/(tf_ball.at(j).z-tf_ball.at(i).z);
                                 temp=temp+tt;
                             }
                             else
                             {
                                 tt=(tf_ball.at(i).y-tf_ball.at(j).y)/(tf_ball.at(i).z-tf_ball.at(j).z);
                                 temp=temp+tt;
                             }
                         }
                     }
                     temp=temp/10.0;
                     flag_temp=true;

                 }

              }
              if (flag_temp)
              {
          float sin_=temp/(sqrt(temp*temp+1));
          float cos_=1.0/(sqrt(temp*temp+1));
          Eigen::MatrixXf m4;
          m4.conservativeResize(4,4);
          m4(0,3) = KINECT_POS_X;
          m4(1,3) = KINECT_POS_Y;
          m4(2,3) = KINECT_POS_Z;
          m4(3,0) = 0; m4(3,1) = 0; m4(3,2) = 0; m4(3,3) = 1;
          m4(0,0) = 0;
          m4(0,1) = sin_;
          m4(0,2) = cos_;
          m4(1,0) = 1;
          m4(1,1) = 0;
          m4(1,2) = 0;
          m4(2,0) = 0;
          m4(2,1) = -cos_;
          m4(2,2) = sin_;
          tf_matrx_ = m4;
          std::string tf_matrix_path   = calibration_path + "/tf_matrix.txt";
          std::ofstream tf_matrix_file(tf_matrix_path.c_str());
          tf_matrix_file << tf_matrx_;
          tf_matrix_file.close();
              }

          }
/*
        if (point_cloud_view && !ball_detection_view && !obstacle_detection_view)
        {//只看点云，不看障碍物
        viewer11->removeAllPointClouds();
        viewer11->removeAllShapes();
        viewer11->addPointCloud<pcl::PointXYZRGBA> (cloud.makeShared(),"sample cloud");
         viewer11->spinOnce (10);
        }
        if (point_cloud_view && obstacle_detection_view)
        {//看点云与障碍物
        viewer11->removeAllPointClouds();
        viewer11->removeAllShapes();
        viewer11->addPointCloud<pcl::PointXYZRGBA> (cloud.makeShared(),"sample cloud");

        for (int i=0;i<num_obstacle_candidate;i++)
        {

            if (obstacle_position[i*2+1]==0||obstacle_position[i*2+1]>6100)
                continue;
            geometry_msgs::Point obs_pp;
            obs_pp.x=obstacle_position[i*2+1];
            obs_pp.y=obstacle_position[i*2+0];

            obstacle_ball_position.obs_pos.push_back(obs_pp);
            obstacle_ball_position.obs_know=true;
//           if(obstacle_position[i*2+1]<6100)
//           {
            printf("number of obstacles left forwa++++rd %d %f %f\n",i,obs_pp.y,obs_pp.x);
            sss<<i;
            string number1=sss.str();
              viewer11->addCube(obstacle_position[i*2+0]-50,obstacle_position[i*2+0]+250,300,400,obstacle_position[i*2+1]-180,obstacle_position[i*2+1]+120,255.0,1.0,1.0,number1.c_str());
  //          viewer11->addSphere(pcl::PointXYZ(obstacle_position[i*2+0],300,obstacle_position[i*2+1]),200,number1.c_str());
  //         viewer11->addCube(obstacle_position[i*2+0]-180,obstacle_position[i*2+0]+180,400,480,obstacle_position[i*2+1]-180,obstacle_position[i*2+1]+180,255.0,1.0,1.0,number1.c_str());
//          }
        }
       if (ball_detection_view && obstacle_ball_position.ball_know)
       {//添加球可视化
           for (int i=0;i<obstacle_ball_position.ball_pos.size();i++)
           {
           sss1<<i+10;
           string number2=sss1.str();
           viewer11->addSphere(pcl::PointXYZ(ball_point.x,ball_point.y,ball_point.z),100,number2.c_str());
           }
           }
         viewer11->spinOnce (10);
        }
*/
        if(!obstacle_detection_view && num_obstacle_candidate)
        {
        for (int i=0;i<num_obstacle_candidate;i++)
        {

            if (obstacle_position[i*2+1]==0||obstacle_position[i*2+1]>6100)
                continue;
            geometry_msgs::Point obs_pp;
            obs_pp.x=obstacle_position[i*2+1];
            obs_pp.y=obstacle_position[i*2+0];

            obstacle_ball_position.obs_pos.push_back(obs_pp);
            obstacle_ball_position.obs_know=true;
        }
        }
       obstacle_ball_position.header.stamp=ros::Time::now();
       obstacle_ball_pub.publish(obstacle_ball_position);
       printf("publish time %f(ms)\n",(dtime()-tend)*1000);
       tend=dtime();
       printf("process time %f(ms)\n",(tend-clocktime0)*1000);
        framecount++;

        if (!viewer_enabled)//viewer_enabled=false, 所以一直进入这里continue就不会执行后面的了
        {
            listener.release(frames);
            continue;
        }

  #ifdef EXAMPLES_WITH_OPENGL_SUPPORT
        protonect_shutdown = protonect_shutdown || viewer.render();
  #endif

        /// [loop end]
        listener.release(frames);
        /** libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100)); */

    }

    cppfile.close();
    ball_time.close();
    obs_time.close();
    time1.close();
    /// [loop end]
    cudaFree(d_cloud);
    cudaFree(d_c);
    cudaFree(d_raw_rgb);
    cudaFree(d_dis_depth);
    cudaFree(d_map_dist);
    cudaFree(d_map_x);
    cudaFree(d_map_yi);
     cudaFree(seg);
    cudaFree(d_seg_table);
    cudaFreeHost(h_register);
    free(ball_position);
    cudaFree(d_table);
    free(h_num_yellow);
    free(obstacle_position);
    cudaFree(pj);
    cudaFree(coeffi);
    free(h_ppj);
    cudaFree(histo_x);
    cudaFree(histo_y);
    cudaFree(histo_z);
    cudaFree(d_kernel);
    cudaFree(result_);
    cudaFree(label);
    cudaFreeHost(h_label);
    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    /// [stop]s
    ///
    dev->stop();
    dev->close();
    /// [stop]

    delete registration;


}
