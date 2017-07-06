#ifndef KINECT_H
#define KINECT_H
//PCL includes
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

//ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>

//nubot includes
//#include "nubot/core/core.hpp"
//#include <nubot_common/BallInfo3d.h>
//#include <nubot_common/ObstaclesInfo3d.h>
#include "colorsegment.h"
//#include <kinect/kinectConfig.h>

#define KINECT_DELTA_BALL_IN_2D 0.02


    bool ball2D_found_ = false;
    bool ball3D_found_;

//    ofstream outfile_;
    int i = 0;
    float x,y,p;


    nubot::ColorSegment::ImageArea    ball_area_;
//the transformation is stored as a (Dim+1)^2 matrix, where the last row is assumed to be [0 ... 0 1].
    vector<float> coeffi_plane;
    Eigen::Vector4f ball_pos3d_;//in robot axis
  //  nubot_common::BallInfo3d      ball_info_;
  //  nubot_common::BallInfo3d      ball_last_see_;//used to calculate ball velocety
    ros::Publisher                    ballinfo_pub_;  //unit: cm
    ros::Subscriber                   pointcloud_sub_;//unit: meter
    void processCallback(const sensor_msgs::PointCloud2 & _point_cloud);
    boost::shared_ptr<pcl::PassThrough<pcl::PointXYZRGBA> >    pass_;
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract_;
    pcl::PCLPointCloud2 point_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA> point_cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_temp_;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_ball_;
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGBA>::Ptr circle2D_model_;
    pcl::ModelCoefficients::Ptr coefficients_sphere_;
    pcl::PointIndices::Ptr inliers_sphere_;
    pcl::SACSegmentation<pcl::PointXYZRGBA> segmenter_sphere_;

//    boost::shared_ptr<dynamic_reconfigure::Server<nubot::kinectConfig> > reconfigure_server_;
//    void ConfigCallback_(nubot::kinectConfig &_config, uint32_t _level);
    bool show_result_;
    std::string node_name_;//used to mark this node
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer11 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PCLVisualizer::Ptr viewer_1;
    bool output_info_;

//    nubot_common::Point3d _point;




    bool calibrate_;
    bool poseCalibration();//only calibrate the three angles of kinect
    ros::Time time_last_;
    ros::Time time_temp_;
    ros::Time time_now_;

//  class ObstaclesProcesser : public nodelet::Nodelet
//    {
//    public:
//        virtual void onInit();
//        ~ObstaclesProcesser();
//    private:
//        bool classify ;
//        bool obstacles3d_found ;

//        ofstream outfile_;
//        int i;
//        std::string node_name_;//used to mark this node
//        void processCallback2(const sensor_msgs::PointCloud2 & _point_cloud);
//        void ComputeCentroid(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud);
//       // void ballpose2d(const nubot_common::OminiVisionInfo & omni_info );
//        ros::Publisher                    Obstacles_pub_;  //unit: cm
//        ros::Subscriber                   worldmodelinfo_sub_;//使用机器人自身的信息
//        ros::Subscriber                   pointcloud_sub_;//unit: meter
//        ros::Subscriber                   ballinfo_sub_;

//        Eigen::Matrix4f tf_matrx;
//        Eigen::Vector4f coefficients_plane;
//        Eigen::Vector4f centroid, centroid1, centroid_;
//        Eigen::Vector4f obstacles_pos3d_;//in robot axis
//        vector<Eigen::Vector4f> obstacles3d_pose;
//        nubot_common::ObstaclesInfo3d      obstacles_info3d_;
//        nubot_common::ObstaclesInfo3d      obstacles_last_see_;//used to calculate obstacles velocety
//        nubot_common::Point3d point_;

//        pcl::PCLPointCloud2 point_cloud;
//        pcl::PointCloud<pcl::PointXYZRGBA> point_cloud_;
//        pcl::PointCloud<pcl::PointXYZRGBA> cloud ,
//                                           cloud_filtered,
//                                           cloud_filtered1,
//                                           cloud_filtered2,
//                                           cloud_filtered3,
//                                           cloud_temp_2;


//        //pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
//        std::vector<pcl::PointIndices> cluster_indices;
//        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
//        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
//     //   pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
//        std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clusters;
//        pcl::PointXYZRGBA min_pt, min_pt1, min_pt2, max_pt, max_pt1, max_pt2;

//        pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr plane2D_model_;

//        boost::shared_ptr<dynamic_reconfigure::Server<nubot::kinectConfig> > reconfigure_server_1;
//        void ConfigCallback_1(nubot::kinectConfig &_config, uint32_t _level);
//        std::string node_name_1;//used to mark this node

//        std::string file_path_;
//        ofstream outfile;//print the centroid

//        ros::Time time_temp;
//        ros::Time time_now;

//        bool show_obstacles_;
//        bool output_obsinfo;
//        boost::shared_ptr<pcl::visualization::PCLVisualizer> mview;
//    };

#endif
