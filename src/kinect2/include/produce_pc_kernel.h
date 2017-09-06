//#include <pcl/visualization/cloud_viewer.h>
#include <string.h> // memcpy
#include <cstdlib>
#include <signal.h>
#include <time.h>
#include <math.h>
//#include <thrust/device_ptr.h>
//#include <algorithm>
#include <vector>
#include <iostream>
using namespace std;
extern "C"{
void registration_kernel(const float* d_dis_depth, const unsigned int* d_raw_rgb, const int* d_map_dist, const float* d_map_x, const int* d_map_yi, const float* cc, float *_cloud, unsigned char *d_table, unsigned char *seg_result, int *d_num_yellow);//, const float* cc, float *_cloud
void region_search(unsigned char *segment_table, int *ball_position, unsigned int *d_output, int number);
void pass_cuda_ball(float *cloud, float *output, float *filter_value);
void filter_obstacle(float *d_cloud, float *output, float *filter_value_obstacle);
void cluster_center(int *d_lable, int *ball_position, vector<int> v);
void cluster_center_(int *d_lable, int *ball_position, vector<int> vv, vector<int> num);
int CCL(unsigned char *d_seg_table, int *d_label, float *ball_position);
int project2D(float *d_cloud, unsigned char *pj, float *aa, float *obstacle_position, unsigned char *d_table, int *histo_x, int *histo_y, int *histo_z, double *d_kernel, size_t kernel_size, unsigned char *result_, int *label, int *h_label);
}
