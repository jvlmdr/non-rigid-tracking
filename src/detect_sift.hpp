#ifndef DETECT_SIFT_HPP_
#define DETECT_SIFT_HPP_

#include <opencv2/core/core.hpp>
#include <vector>
#include "sift_feature.hpp"

struct SiftOptions {
  int max_num_features;
  int num_octave_layers;
  double contrast_threshold;
  double edge_threshold;
  double sigma;
};

void extractFeatures(const cv::Mat& image,
                     std::vector<SiftFeature>& features,
                     const SiftOptions& options);

#endif
