#include "computeedmap.h"

cv::Mat computediscontinuitymap(cv::Mat& depth){
        cv::Size image_size(depth.size());
        cv::Mat discontinuity_map = cv::Mat::zeros(depth.size(), CV_32FC1);  
        cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(3,3));
        cv::Mat depth_without_nans(image_size, CV_32FC1);
        cv::threshold(depth, depth_without_nans, 0, 1u,
                    cv::THRESH_TOZERO);
        cv::Mat dilate_image(image_size, CV_32FC1);
        cv::dilate(depth_without_nans, dilate_image, element);
        dilate_image -= depth_without_nans;
        cv::Mat erode_image(image_size, CV_32FC1);
        cv::erode(depth_without_nans, erode_image, element);
        erode_image = depth_without_nans - erode_image;

        cv::Mat max_image(image_size, CV_32FC1);
        cv::max(dilate_image, erode_image, max_image);

        cv::Mat ratio_image(image_size, CV_32FC1);
        cv::divide(max_image, depth_without_nans, ratio_image);
        cv::threshold(ratio_image, discontinuity_map,
                    0.01, 1u,
                    cv::THRESH_BINARY);

        if (true) {
        cv::Mat element = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(2u * 1 + 1u,
                        2u * 1 + 1u),
            cv::Point(1,1));


        cv::morphologyEx(discontinuity_map, discontinuity_map, cv::MORPH_OPEN,
                            element);
        }
        return discontinuity_map;

}
cv::Mat computedistancemap(cv::Mat& depth_map){

        cv::Mat distance_map = cv::Mat::zeros(480, 640, CV_32FC1);  
        distance_map.setTo(cv::Scalar(0.0f));
        size_t kernel_size = 3;
        size_t n_kernels = kernel_size * kernel_size - 1u;


        for (size_t i = 0u; i < n_kernels + 1u; ++i) {
            if (i == n_kernels / 2u) {
                continue;
            }
            cv::Mat kernel = cv::Mat::zeros(kernel_size, kernel_size, CV_32FC1);
            kernel.at<float>(i) = -1.0f;
            kernel.at<float>(n_kernels / 2u) = 1.0f;

            // Compute the filtered images.
            cv::Mat filtered_image(depth_map.size(), CV_32FC3);
            cv::filter2D(depth_map, filtered_image, CV_32FC3, kernel);

            std::vector<cv::Mat> channels(3);
            cv::split(filtered_image, channels);
            cv::Mat distance_map1(depth_map.size(), CV_32FC1);

            if (false) {
                cv::Mat mask_0 = cv::Mat(channels[0] == channels[0]);
                cv::Mat mask_1 = cv::Mat(channels[1] == channels[1]);
                cv::Mat mask_2 = cv::Mat(channels[2] == channels[2]);
                mask_0.convertTo(mask_0, CV_32FC1);
                mask_1.convertTo(mask_1, CV_32FC1);
                mask_2.convertTo(mask_2, CV_32FC1);
                distance_map1 = mask_0.mul(channels[0].mul(channels[0])) +
                                mask_1.mul(channels[1].mul(channels[1])) +
                                mask_2.mul(channels[2].mul(channels[2]));
            } else {
                distance_map1 = channels[0].mul(channels[0]) +
                                channels[1].mul(channels[1]) +
                                channels[2].mul(channels[2]);
            }

            if (false) {
                cv::Mat mask = cv::Mat(distance_map == distance_map);
                mask.convertTo(mask, CV_32FC1);
                distance_map = mask.mul(distance_map);
            }
            cv::max(distance_map, distance_map1, distance_map);
        }
        return distance_map;
}

void computetauandphiemap(cv::Mat& depth, cv::Mat& depth_map, cv::Mat& normal_map, cv::Mat* phis, cv::Mat* taus)
{
        cv::Mat localphis = cv::Mat::zeros(depth.size(), CV_32FC1);
        cv::Mat localtaus = cv::Mat::zeros(depth.size(), CV_32FC1);

        localphis.setTo(cv::Scalar(10.0f));

        size_t kernel_size = 5;
        size_t  n_kernels = 5 * 5 -  1u;

        for (size_t i = 0u; i < n_kernels + 1u; i += static_cast<size_t>(i % kernel_size == kernel_size) * kernel_size + 1) 
        {
            if (i == n_kernels / 2u)
            {
                continue;
            }
            cv::Mat difference_kernel =
                cv::Mat::zeros(kernel_size, kernel_size, CV_32FC1);
            difference_kernel.at<float>(i) = 1.0f;

            difference_kernel.at<float>(n_kernels / 2u) = -1.0f;

            cv::Mat difference_map(depth_map.size(), CV_32FC3);
            cv::filter2D(depth_map, difference_map, CV_32FC3, difference_kernel);

            cv::Mat difference_times_normal(depth_map.size(), CV_32FC3);
            difference_times_normal = difference_map.mul(normal_map);
            std::vector<cv::Mat> channels(3);
            cv::split(difference_times_normal, channels);
            cv::Mat vector_projection(depth_map.size(), CV_32FC1);
            vector_projection = channels[0] + channels[1] + channels[2];

            cv::max(localtaus, vector_projection, localtaus);


            cv::Mat normal_kernel = cv::Mat::zeros(kernel_size, kernel_size, CV_32FC1);
            normal_kernel.at<float>(i) = 1.0f;

            cv::Mat filtered_normal_image = cv::Mat::zeros(normal_map.size(), CV_32FC3);
            cv::filter2D(normal_map, filtered_normal_image, CV_32FC3, normal_kernel);
            normal_map.copyTo(filtered_normal_image,
                            filtered_normal_image != filtered_normal_image);

            // TODO(ff): Create a function for this mulitplication and projections.
            cv::Mat normal_times_filtered_normal(depth_map.size(), CV_32FC3);
            normal_times_filtered_normal = normal_map.mul(filtered_normal_image);
            filtered_normal_image.copyTo(
                normal_times_filtered_normal,
                normal_times_filtered_normal != normal_times_filtered_normal);
            std::vector<cv::Mat> normal_channels(3);
            cv::split(normal_times_filtered_normal, normal_channels);
            cv::Mat normal_vector_projection(depth_map.size(), CV_32FC1);
            normal_vector_projection =
                normal_channels[0] + normal_channels[1] + normal_channels[2];

            // normal_vector_projection.setTo(1.0, greaterthanzeromask);

            cv::min(localphis, normal_vector_projection, localphis);

        }
        if (true) {
            constexpr float kMaxBinaryValue = 1.0f;
            cv::threshold(localphis, localphis,
                            0.97, kMaxBinaryValue,
                            cv::THRESH_BINARY);
        }

        *phis = localphis;
        *taus = localtaus;

}

cv::Mat computefinaledgemap(cv::Mat& depth, cv::Mat& depth_map, cv::Mat& normal_map){

    cv::Mat phis = cv::Mat::zeros(depth.size(), CV_32FC1);
    cv::Mat taus = cv::Mat::zeros(depth.size(), CV_32FC1);
    cv::Mat discontinuity_map = computediscontinuitymap(depth);
    cv::Mat distance_map = computedistancemap(depth_map);
    computetauandphiemap(depth, depth_map, normal_map, &phis, &taus);
    cv::Mat edge_map = cv::Mat::zeros(depth.size(), CV_32FC1);
    cv::Mat distance_discontinuity_map( cv::Size(distance_map.cols, distance_map.rows), CV_32FC1);
    cv::threshold(distance_map + discontinuity_map+taus, distance_discontinuity_map,1.0, 1.0, cv::THRESH_TRUNC);
    edge_map = phis - distance_discontinuity_map;
    if (false) {
        cv::Mat element = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(2u * 1 + 1u, 2u * 1 + 1u),
            cv::Point(1, 1));
        cv::morphologyEx(edge_map, edge_map, cv::MORPH_OPEN,
                            element);
        }

    if (true) {
        constexpr float kMaxBinaryValue = 1.0f;
        cv::threshold(edge_map, edge_map,
                        0.99, kMaxBinaryValue,
                        cv::THRESH_BINARY);
    }
    return edge_map;
}