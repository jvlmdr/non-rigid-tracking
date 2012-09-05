#include "detect_sift.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <glog/logging.h>

SiftPosition keypointToSiftPosition(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return SiftPosition(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

void extractFeatures(const cv::Mat& image,
                     std::vector<SiftFeature>& features,
                     const SiftOptions& options) {
  // Clear list.
  features.clear();

  // SIFT settings.
  cv::SIFT sift(options.max_num_features, options.num_octave_layers,
      options.contrast_threshold, options.edge_threshold, options.sigma);

  // Extract SIFT keypoints and descriptors.
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift(image, cv::noArray(), keypoints, descriptors, false);

  // Ensure that descriptors are 32-bit floats.
  CHECK(descriptors.type() == cv::DataType<float>::type);

  int num_features = keypoints.size();

  // Convert to features.
  for (int i = 0; i < num_features; i += 1) {
    SiftFeature feature;
    feature.position = keypointToSiftPosition(keypoints[i]);

    // Copy descriptor contents.
    cv::Mat row = descriptors.row(i);
    feature.descriptor.data.clear();
    std::copy(row.begin<float>(), row.end<float>(),
        std::back_inserter(feature.descriptor.data));

    // Add to list.
    features.push_back(feature);
  }
}
