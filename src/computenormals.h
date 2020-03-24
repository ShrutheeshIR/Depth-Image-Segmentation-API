#ifndef COMPUTE_NORMALS_H
#define COMPUTE_NORMALS_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/rgbd.hpp>
#include <algorithm>
// #include <omp.h> 


using namespace cv;


void computeCovariance(const cv::Mat& neighborhood, const cv::Vec3f& mean, const size_t neighborhood_size, cv::Mat* covariance);
size_t findNeighborhood(const cv::Mat& depth_map, const size_t window_size, const float max_distance, const size_t x, const size_t y, cv::Mat* neighborhood, cv::Vec3f* mean);
void computeOwnNormals(const cv::Mat& depth_map, cv::Mat* normals, int no);                        


#endif