#include "segmentedgemap.h"
#include "computeedmap.h"
#include "computenormals.h"
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

using namespace std;
using namespace cv;

void segmentdepthimage(Mat& depth_image, Mat& camera_matrix);
