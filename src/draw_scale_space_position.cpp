#include "draw_scale_space_position.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

void drawScaleSpacePosition(cv::Mat& image,
                            const ScaleSpacePosition& feature,
                            int radius,
                            const cv::Scalar& color,
                            int thickness) {
  cv::Point2d point = feature.point();
  double r = radius * feature.scale;

  cv::Point2d pt1 = point - cv::Point2d(r + 1, r + 1);
  cv::Point2d pt2 = point + cv::Point2d(r + 1, r + 1);

  cv::rectangle(image, pt1, pt2, color, thickness);
}
