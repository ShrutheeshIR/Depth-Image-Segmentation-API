#include "computenormals.h"

void computeCovariance(const cv::Mat& neighborhood, const cv::Vec3f& mean, const size_t neighborhood_size, cv::Mat* covariance){
    *covariance = cv::Mat::zeros(3, 3, CV_32F);
    for (size_t i = 0u; i < neighborhood_size; ++i) 
    {
        cv::Vec3f point;
        for (size_t row = 0u; row < neighborhood.rows; ++row) 
            point[row] = neighborhood.at<float>(row, i) - mean[row];
                covariance->at<float>(0, 0) += point[0] * point[0];
        covariance->at<float>(0, 1) += point[0] * point[1];
        covariance->at<float>(0, 2) += point[0] * point[2];
        covariance->at<float>(1, 1) += point[1] * point[1];
        covariance->at<float>(1, 2) += point[1] * point[2];
        covariance->at<float>(2, 2) += point[2] * point[2];
    }
    // Assign the symmetric elements of the covariance matrix.
    covariance->at<float>(1, 0) = covariance->at<float>(0, 1);
    covariance->at<float>(2, 0) = covariance->at<float>(0, 2);
    covariance->at<float>(2, 1) = covariance->at<float>(1, 2);
}

size_t findNeighborhood(const cv::Mat& depth_map, const size_t window_size,
                        const float max_distance, const size_t x,
                        const size_t y, cv::Mat* neighborhood,
                        cv::Vec3f* mean) {

  size_t neighborhood_size = 0u;
  *neighborhood = cv::Mat::zeros(3, window_size * window_size, CV_32FC1);
  cv::Vec3f mid_point = depth_map.at<cv::Vec3f>(y, x);
  for (size_t y_idx = 0u; y_idx < window_size; ++y_idx) {
    const int y_filter_idx = y + y_idx - window_size / 2u;
    if (y_filter_idx < 0 || y_filter_idx >= depth_map.rows) {
      continue;
    }
    for (size_t x_idx = 0u; x_idx < window_size; ++x_idx) {
      const int x_filter_idx = x + x_idx - window_size / 2u;
      if (x_filter_idx < 0 || x_filter_idx >= depth_map.cols) {
        continue;
      }

      cv::Vec3f filter_point =
          depth_map.at<cv::Vec3f>(y_filter_idx, x_filter_idx);

      // Compute Euclidean distance between filter_point and mid_point.
      const cv::Vec3f difference = mid_point - filter_point;
      const float euclidean_dist = cv::sqrt(difference.dot(difference));
      if (euclidean_dist < max_distance) {
        // Add the filter_point to neighborhood set.
        for (size_t coordinate = 0u; coordinate < 3u; ++coordinate) {
          neighborhood->at<float>(coordinate, neighborhood_size) =
              filter_point[coordinate];
        }
        ++neighborhood_size;
        *mean += filter_point;
      }
    }
  }
  *mean /= static_cast<float>(neighborhood_size);
  return neighborhood_size;
}

void computeOwnNormals(const cv::Mat& depth_map, cv::Mat* normals, int no) {

  cv::Mat neighborhood =
      cv::Mat::zeros(3, no * no, CV_32FC1);
  cv::Mat eigenvalues;
  cv::Mat eigenvectors;
  cv::Mat covariance(3, 3, CV_32FC1);
  covariance = cv::Mat::zeros(3, 3, CV_32FC1);
  cv::Vec3f mean;
  cv::Vec3f mid_point;

  constexpr float float_nan = std::numeric_limits<float>::quiet_NaN();
  // #pragma omp parallel for private(neighborhood, eigenvalues, eigenvectors, covariance, mean, mid_point)
  for (size_t y = 0u; y < depth_map.rows; ++y) {
    // std::cout<<"KYGYTVM";
    for (size_t x = 0u; x < depth_map.cols; ++x) {
      mid_point = depth_map.at<cv::Vec3f>(y, x);
      if (cvIsNaN(mid_point[0]) || cvIsNaN(mid_point[1]) ||
          cvIsNaN(mid_point[2]) || (mid_point[2] == 0.0)) {
        normals->at<cv::Vec3f>(y, x) =
            cv::Vec3f(float_nan, float_nan, float_nan);
        continue;
      }
      const float max_distance =
          0.05 * mid_point[2];
      mean = cv::Vec3f(0.0f, 0.0f, 0.0f);

      const size_t neighborhood_size =
          findNeighborhood(depth_map, no, max_distance, x, y,
                           &neighborhood, &mean);
      if (neighborhood_size > 1u) {
        computeCovariance(neighborhood, mean, neighborhood_size, &covariance);
        // Compute Eigen vectors.
        cv::eigen(covariance, eigenvalues, eigenvectors);
        // Get the Eigenvector corresponding to the smallest Eigenvalue.
        constexpr size_t n_th_eigenvector = 2u;
        for (size_t coordinate = 0u; coordinate < 3u; ++coordinate) {
          normals->at<cv::Vec3f>(y, x)[coordinate] =
              eigenvectors.at<float>(n_th_eigenvector, coordinate);
        }
        // Re-Orient normals to point towards camera.
        if (normals->at<cv::Vec3f>(y, x)[2] > 0.0f) {
          normals->at<cv::Vec3f>(y, x) = -normals->at<cv::Vec3f>(y, x);
        }
      } else {
        normals->at<cv::Vec3f>(y, x) =
            cv::Vec3f(float_nan, float_nan, float_nan);
      }
    }
  }
}