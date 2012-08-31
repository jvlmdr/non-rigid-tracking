#ifndef DRAW_SIFT_POSITION_HPP_
#define DRAW_SIFT_POSITION_HPP_

#include <opencv2/core/core.hpp>
#include "sift_position.hpp"

void drawSiftPosition(cv::Mat& image,
                      const SiftPosition& feature,
                      const cv::Scalar& color,
                      int line_thickness);

#endif
