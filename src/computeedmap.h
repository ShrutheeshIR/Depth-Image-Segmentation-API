#ifndef COMPUTE_EDGE_MAP_H
#define COMPUTE_EDGE_MAP_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/rgbd.hpp>
#include <algorithm>
#include <omp.h> 


using namespace cv;


cv::Mat computediscontinuitymap(cv::Mat& depth);
cv::Mat computedistancemap(cv::Mat& depth_map);
void computetauandphiemap(cv::Mat& depth, cv::Mat& depth_map, cv::Mat& normal_map, cv::Mat* phis, cv::Mat* taus);
cv::Mat computefinaledgemap(cv::Mat& depth, cv::Mat& depth_map, cv::Mat& normal_map);


#endif