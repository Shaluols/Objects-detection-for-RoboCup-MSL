#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <signal.h>
#include <string.h> // memcpy
#include <fstream>
#include <math.h>
#include <cmath>
#include <sys/time.h>
using namespace std;

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

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
#include "colorsegment.h"
#include "nubot_common/object_info.h"
#include "geometry_msgs/Point.h"
#include <LaterMethods.h>
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
#include "viewer.h"
#include <pcl/visualization/cloud_viewer.h>
#endif

#define KINECT_POS_X 0.12
#define KINECT_POS_Y -0.07
#define KINECT_POS_Z 0.49
#define KINECT_BALL_SIZE_THRESHOLD 30

class Protonect
{
public:
    double dtime();
    void cicle();

    Protonect();
    ~Protonect();

public:

    size_t imageSize, grayBytes,table_size;
    size_t rgbsize, kernel_size;
    float* d_dis_depth, *d_map_x, *d_cloud, *d_c, *ball_position, *obstacle_position;
    int* d_map_dist, *d_map_yi, *d_label, *h_num_yellow;
    unsigned int* d_raw_rgb, *d_register;
    unsigned char *h_register, *d_seg_table, *seg, *d_table, *pj;
    int agent, num_ball_candidate,num_obstacle_candidate;
    char * home, * environment;
    std::stringstream sh, ss, sss,sss1;
    std::string calibration_path, color_table_path, tf_matrix_path;
    Eigen::Matrix4f tf_matrx_;
    nubot::ColorSegment *color_segmenter_;

    float *coeffi;
    unsigned char *h_ppj, *result_;
    int *histo_x, *histo_y, *histo_z, *label, *h_label;
    double *d_kernel;
    vector<double> kernel;
    bool cali_tf, color_calibrating,view_color_seg, point_cloud_view;
    bool ball_detection_view, obstacle_detection_view;
    vector<geometry_msgs::Point> tf_ball;
    pcl::PointXYZRGBA ball_point,ball_point_temp;
    cv::Mat rgb_image;

    ros::NodeHandle nh;
    ros::Publisher obstacle_ball_pub;
    libfreenect2::Freenect2 freenect2;
     float *ttime, *ttime1;
     double tend;

};



