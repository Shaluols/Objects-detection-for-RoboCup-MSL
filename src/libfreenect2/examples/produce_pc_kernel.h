//#include <pcl/visualization/cloud_viewer.h>
extern "C"{
//void vectorAdd1(const float *A, const float *B, float *C, int numElements);
//void produce_pc_kernel(const float* _depth, const float* _rgb, const int _width, const int _height, int* table_x,  int* table_y, float *_cloud);
//void produce_pc_kernel(const float* _depth, const float* _rgb, const int _width, const int _height, float* cx, float* cy, float* fx, float* fy, float *_cloud);
void produce_pc_kernel(const float* _depth, const int _width, const int _height, float* cx, float* cy, float* fx, float* fy, float *X, float *Y, float *Z);
void produce_pc_kernel2(const float* _depth, const float* _rgba, const int _width, const int _height, const float* cx, float *_cloud);
void registration_kernel(const float* d_dis_depth, const unsigned int* d_raw_rgb, unsigned int* d_register, const int* d_map_dist, const float* d_map_x, const int* d_map_yi, const float* cc, float *_cloud);//, const float* cc, float *_cloud
//(const float* _depth, const float* _rgb, const int _width, const int _height, const int table_x[], const int table_y[], float *_cloud)
//void produce_pc_kernel(const float* _depth, const int _width, const int _height, float* cx, float *X, float *Y, float *Z);
}
