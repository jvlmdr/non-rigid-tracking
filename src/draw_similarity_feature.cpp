#include "draw_similarity_feature.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

const double NUM_STDDEV = 2.;
const int LINE_THICKNESS = 1;

void drawSimilarityFeature(cv::Mat& image,
                           const SimilarityFeature& feature,
                           const cv::Scalar& color) {
  double radius = NUM_STDDEV * SIFT_SIZE_TO_SIGMA * feature.size;

  cv::Point2d c(feature.x, feature.y);
  cv::Point2d j(radius * std::cos(feature.theta),
                radius * std::sin(feature.theta));

  cv::circle(image, c, radius, color, LINE_THICKNESS);
  cv::line(image, c, c + j, color, LINE_THICKNESS);
}
