#ifndef SEGMENT_EDGE_MAP_H
#define SEGMENT_EDGE_MAP_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/rgbd.hpp>
#include <algorithm>

using namespace cv;


struct Segment {
  std::vector<cv::Vec3f> points;
  std::vector<cv::Vec3f> normals;
  std::vector<cv::Vec3f> original_colors;
  std::set<size_t> label;
  std::set<size_t> instance_label;
  std::set<size_t> semantic_label;
  cv::Mat segment_mask;
};



static bool segment_sorter(Segment& s1, Segment& s2);



void inpaintImage(const cv::Mat& depth_image, const cv::Mat& edge_map, const cv::Mat& label_map, cv::Mat* inpainted);
void generateRandomColorsAndLabels( size_t contours_size, std::vector<cv::Scalar>* colors, std::vector<int>* labels);
void labelMap(const cv::Mat& rgb_image, const cv::Mat& depth_image, const cv::Mat& depth_map, const cv::Mat& edge_map, const cv::Mat& normal_map, cv::Mat& cam_matrix, cv::Mat* labeled_map, std::vector<cv::Mat>* segment_masks, std::vector<Segment>* segments);


#endif