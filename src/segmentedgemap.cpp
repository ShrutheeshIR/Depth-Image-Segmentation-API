#include "segmentedgemap.h"

std::vector<cv::Scalar> colors_;
std::vector<int> labels_;


bool segment_sorter(Segment& s1, Segment& s2){
    return s1.points.size() < s2.points.size();
}

void inpaintImage(const cv::Mat& depth_image, const cv::Mat& edge_map, const cv::Mat& label_map, cv::Mat* inpainted) {

  cv::Mat gray_edge;
  cv::cvtColor(label_map, gray_edge, CV_BGR2GRAY);

  cv::Mat mask = cv::Mat::zeros(edge_map.size(), CV_8UC1);
  cv::bitwise_and(depth_image == depth_image, gray_edge == 0, mask);
  constexpr double kInpaintRadius = 1.0;
  cv::inpaint(label_map, mask, *inpainted, kInpaintRadius,1);
}

void generateRandomColorsAndLabels( size_t contours_size, std::vector<cv::Scalar>* colors, std::vector<int>* labels) {
  if (colors_.size() < contours_size) {
    colors_.reserve(contours_size);
    for (size_t i = colors_.size(); i < contours_size; ++i) {
      colors_.push_back(
          cv::Scalar(255 * (rand() / static_cast<float>(RAND_MAX)),
                     255 * (rand() / static_cast<float>(RAND_MAX)),
                     255 * (rand() / static_cast<float>(RAND_MAX))));
      labels_.push_back(i);
    }
  }
  *colors = colors_;
  *labels = labels_;
}

void labelMap(const cv::Mat& rgb_image, const cv::Mat& depth_image, const cv::Mat& depth_map, const cv::Mat& edge_map, const cv::Mat& normal_map, cv::Mat& cam_matrix, cv::Mat* labeled_map, std::vector<cv::Mat>* segment_masks, std::vector<Segment>* segments) {

    constexpr size_t kMaskValue = 255u;

    cv::Mat original_depth_map;
    cv::rgbd::depthTo3d(depth_image, cam_matrix,
                        original_depth_map);

    cv::Mat output = cv::Mat::zeros(depth_image.size(), CV_8UC3);
        // TODO(ff): Move to method.
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat edge_map_8u;
        edge_map.convertTo(edge_map_8u, CV_8U);
        static const cv::Point kContourOffset = cv::Point(0, 0);
        cv::findContours(edge_map_8u, contours, hierarchy,
                        cv::RETR_TREE, /*cv::RETR_CCOMP*/
                        CV_CHAIN_APPROX_NONE, kContourOffset);
        std::vector<cv::Scalar> colors;
        std::vector<int> labels;
        generateRandomColorsAndLabels(contours.size(), &colors, &labels);
        for (size_t i = 0u; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        constexpr int kNoParentContour = -1;
        if (area < 200) {
            const int parent_contour = hierarchy[i][3];
            if (parent_contour == kNoParentContour) {
            // Assign black color to areas that have no parent contour.
            colors[i] = cv::Scalar(0, 0, 0);
            labels[i] = -1;
            drawContours(edge_map_8u, contours, i, cv::Scalar(0u), CV_FILLED, 8,
                            hierarchy);
            } else {
            if (hierarchy[i][0] == -1 && hierarchy[i][1] == -1) {
                // Assign the color of the parent contour.
                colors[i] = colors[parent_contour];
                labels[i] = labels[parent_contour];
            } else {
                colors[i] = cv::Scalar(0, 0, 0);
                labels[i] = -1;
                drawContours(edge_map_8u, contours, i, cv::Scalar(0u), CV_FILLED,
                            8, hierarchy);
            }
            }
        }
        }


        cv::Mat output_labels =
            cv::Mat(depth_image.size(), CV_32SC1, cv::Scalar(0));
        for (size_t i = 0u; i < contours.size(); ++i) {
        drawContours(output, contours, i, cv::Scalar(colors[i]), CV_FILLED, 8,
                        hierarchy);
        drawContours(output_labels, contours, i, cv::Scalar(labels[i]),
                        CV_FILLED, 8, hierarchy);
        drawContours(edge_map_8u, contours, i, cv::Scalar(0u), 1, 8, hierarchy);
        }


        output.setTo(cv::Scalar(0, 0, 0), edge_map_8u == 0u);
        output_labels.setTo(-1, edge_map_8u == 0u);
        // Create a map of all the labels.
        std::map<size_t, size_t> labels_map;
        size_t value = 0u;
        for (size_t i = 0u; i < labels.size(); ++i) {
        if (labels[i] >= 0) {

            // Create a new map if label is not yet in keys.
            if (labels_map.find(labels[i]) == labels_map.end()) {
            labels_map[labels[i]] = value;
            ++value;
            }
        }
        }
        segments->resize(labels_map.size());
        segment_masks->resize(labels_map.size());
        for (Segment& segment : *segments) {
        segment.segment_mask = cv::Mat(depth_image.size(), CV_8UC1, cv::Scalar(0));
        }
        for (size_t x = 0u; x < output_labels.cols; ++x) {
        for (size_t y = 0u; y < output_labels.rows; ++y) {
            int32_t label = output_labels.at<int32_t>(y, x);
            // Check if edge point and assign the nearest neighbor label.
            const bool is_edge_point = edge_map_8u.at<uint8_t>(y, x) == 0u &&
                                        depth_image.at<float>(y, x) > 0.0f;
            if (is_edge_point) {
            // We assign edgepoints by default to -1.
            label = -1;
            const cv::Vec3f& edge_point = depth_map.at<cv::Vec3f>(y, x);
            constexpr double kMinNearestNeighborDistance = 0.05;
            double min_dist = kMinNearestNeighborDistance;
            constexpr int kFilterSizeHalfFloored = 4u;
            for (int i = -kFilterSizeHalfFloored; i <= kFilterSizeHalfFloored;
                    ++i) {
                if (static_cast<int>(x) + i < 0) {
                continue;
                }
                if (static_cast<int>(x) + i >= output_labels.cols) {
                break;
                }
                for (int j = -kFilterSizeHalfFloored; j <= kFilterSizeHalfFloored;
                    ++j) {
                if (static_cast<int>(y) + j < 0 || (i == 0 && j == 0)) {
                    continue;
                }
                if (static_cast<int>(y) + j >= output_labels.rows) {
                    break;
                }
                const cv::Vec3f filter_point =
                    depth_map.at<cv::Vec3f>(y + j, x + i);
                const double dist = cv::norm(edge_point - filter_point);
                if (dist >= min_dist) {
                    continue;
                }
                const bool filter_point_is_edge_point =
                    edge_map_8u.at<uint8_t>(y + j, x + i) == 0u &&
                    depth_image.at<float>(y + j, x + i) > 0.0f;
                if (!filter_point_is_edge_point) {
                    const int label_tmp = output_labels.at<int32_t>(y + j, x + i);
                    if (label_tmp < 0) {
                    continue;
                    }
                    min_dist = dist;
                    label = label_tmp;
                    output_labels.at<int32_t>(y, x) = label;
                }
                }
            }
            if (label > 0) {
                output.at<cv::Vec3b>(y, x) = cv::Vec3b(
                    colors[label][0], colors[label][1], colors[label][2]);
            }
            }
            if (label < 0) {
            continue;
            } else {
            // Append vectors from depth_map and normals from normal_map to
            // vectors of segments.
            cv::Vec3f point = original_depth_map.at<cv::Vec3f>(y, x);
            cv::Vec3f normal = normal_map.at<cv::Vec3f>(y, x);
            cv::Vec3b original_color = rgb_image.at<cv::Vec3b>(y, x);
            cv::Vec3f color_f;
            constexpr bool kUseOriginalColors = true;
            if (kUseOriginalColors) {
                color_f = cv::Vec3f(static_cast<float>(original_color[0]),
                                    static_cast<float>(original_color[1]),
                                    static_cast<float>(original_color[2]));
            } else {
                color_f = cv::Vec3f(static_cast<float>(colors[label][0]),
                                    static_cast<float>(colors[label][1]),
                                    static_cast<float>(colors[label][2]));
            }
            std::vector<cv::Vec3f> rgb_point_with_normals{point, normal,
                                                            color_f};
            Segment& segment = (*segments)[labels_map.at(label)];
            segment.points.push_back(point);
            segment.normals.push_back(normal);
            segment.original_colors.push_back(color_f);
            segment.label.insert(label);
            segment.segment_mask.at<uint8_t>(y, x) = kMaskValue;
            }
        }
        }



    // Remove small segments from segments vector.
    for (size_t i = 0u; i < segments->size();) {
    if ((*segments)[i].points.size() < 200) {
        segments->erase(segments->begin() + i);
        } else {
            ++i;
        }
    }



    inpaintImage(depth_image, edge_map, output, &output);

    if(false){
        static const std::string kWindowName = "LabelMap";
        cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
        imshow(kWindowName, output);
        cv::waitKey(1);
    }
    *labeled_map = output;
    // std::cout<< segments->begin()->segment_mask;
    // segment_sorter(segments->begin());
    std::sort(segments->begin(), segments->end(), &segment_sorter);

}