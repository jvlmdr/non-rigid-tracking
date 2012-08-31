#include "draw_sift_position.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

const double NUM_STDDEV = 2.;

void drawSiftPosition(cv::Mat& image,
                      const SiftPosition& feature,
                      const cv::Scalar& color,
                      int line_thickness) {
  double radius = NUM_STDDEV * SIFT_SIZE_TO_SIGMA * feature.size;

  cv::Point2d c(feature.x, feature.y);
  cv::Point2d j(radius * std::cos(feature.theta),
                radius * std::sin(feature.theta));

  cv::circle(image, c, radius, color, line_thickness);
  cv::line(image, c, c + j, color, line_thickness);
}
