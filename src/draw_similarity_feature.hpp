#ifndef DRAW_SIMILARITY_FEATURE_HPP_
#define DRAW_SIMILARITY_FEATURE_HPP_

#include <opencv2/core/core.hpp>
#include "similarity_feature.hpp"

void drawSimilarityFeature(cv::Mat& image,
                           const SimilarityFeature& feature,
                           const cv::Scalar& color);

#endif
