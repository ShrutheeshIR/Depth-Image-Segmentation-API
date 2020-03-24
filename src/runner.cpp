#include "boostseg.h"

int main()
{
    Mat depth1, depth;
    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat rescaled_depth, dilated_rescaled_depth, bw_image, mask, edge_map, imag, depth2;

    double min, max;
    double ogdmin, ogdmax;
    camera_matrix.at<float>(0, 0) = 585.0f;
    camera_matrix.at<float>(0, 2) = 320.0f;
    camera_matrix.at<float>(1, 1) = 585.0f;
    camera_matrix.at<float>(1, 2) = 240.0f;
    camera_matrix.at<float>(2, 2) = 1.0f;
    const float fx = camera_matrix.at<float>(0, 0);
    const float fy = camera_matrix.at<float>(1, 1);
    const float cx = camera_matrix.at<float>(0, 2);
    const float cy = camera_matrix.at<float>(1, 2);


    for(int iteratorindex = 400; iteratorindex<500;iteratorindex++)
    {

        char ss[7];
        std::string str = "/home/olorin/Desktop/IISc/TSDF/tsdf-fusion-python/data/frame-";
        snprintf (ss, 7, "%06d", iteratorindex);
        str.append(ss);
        str.append(".depth.png");

        depth1 = imread(str, 2|4);

        boostseg BS;
        BS.segmentdepthimage(depth1, camera_matrix);
    }

}