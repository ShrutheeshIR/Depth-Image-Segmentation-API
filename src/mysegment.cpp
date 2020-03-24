#include "mysegment.hpp"
using namespace std;
using namespace cv;

namespace bv
{
    int segmentdepthimage(Mat& depth_image, Mat& camera_matrix, OutputArray imedge){



        // else{
        //     CV_ERROR(CV_StsBadArg, depth_image.size());
        // }


        double ogdmin, ogdmax;
        double min, max;
        cv::Mat rescaled_depth, depth;
        rescaled_depth = cv::Mat::zeros(depth_image.size(), CV_32FC1);
        cv::rgbd::rescaleDepth(depth_image, CV_32FC1, rescaled_depth);
        cv::minMaxLoc(rescaled_depth, &ogdmin, &ogdmax);

        double kZeroValue = ogdmax;
        cv::Mat nan_mask = rescaled_depth != rescaled_depth;
        cv::Mat yes_mask = rescaled_depth == rescaled_depth;
        rescaled_depth.setTo(kZeroValue, nan_mask);
        cv::Mat greater_mask = rescaled_depth > ogdmax*1/2;
        cv::Mat lesser_mask = rescaled_depth <= ogdmax*1/2;
        rescaled_depth.setTo(kZeroValue, greater_mask);
        medianBlur(rescaled_depth, depth, 5);
        static constexpr size_t kNormalImageWidth = 640u;
        static constexpr size_t kNormalImageHeight = 480u;
        
        cv::Size image_size(kNormalImageWidth, kNormalImageHeight);
        cv::Mat depth_map(image_size, CV_32FC3);
        cv::Mat rgbimage(image_size, CV_32FC3);
        cv::rgbd::depthTo3d(depth, camera_matrix,  depth_map);
        cv::Mat normals(image_size, CV_32FC3, 0.0f);

        computeOwnNormals(depth_map, &normals, 11);
        cv::Mat edge_map = computefinaledgemap(depth, depth_map, normals);
        // std::cout<<edge_map;
        // Canny(edge_map,imedge,100,200);



        std::vector<Segment> segments;
        std::vector<cv::Mat> segment_masks;
        const std::vector<cv::Mat> segmen;

        cv::Mat label_map(edge_map.size(), CV_32FC1);
        
        labelMap(rgbimage, rescaled_depth, depth_map, edge_map, normals, camera_matrix, &label_map, &segment_masks, &segments);

        std::cout<<segments[10].segment_mask.type();
        
        Mat finalsegmentmap = cv::Mat::zeros(image_size, 0);

        for(std::vector<int>::size_type i = 0; i != segments.size(); i++) {
            Mat seg = segments[i].segment_mask;
            Mat segmask = seg>0;
            finalsegmentmap.setTo(i+1, segmask);


        }
        finalsegmentmap.copyTo(imedge);
        return segments.size();
    }
}