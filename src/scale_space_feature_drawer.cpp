#include "scale_space_feature_drawer.hpp"

ScaleSpaceFeatureDrawer::ScaleSpaceFeatureDrawer(
    const ScaleSpacePosition& position, int radius) : position_(position),
                                                      radius_(radius) {}

ScaleSpaceFeatureDrawer::~ScaleSpaceFeatureDrawer() {}

void ScaleSpaceFeatureDrawer::draw(cv::Mat& image,
                                   const cv::Scalar& color,
                                   int thickness) const {
  cv::Point2d point = position_.point();
  double radius = radius_ * position_.scale;

  cv::Point2d pt1 = point - cv::Point2d(radius + 1, radius + 1);
  cv::Point2d pt2 = point + cv::Point2d(radius + 1, radius + 1);

  cv::rectangle(image, pt1, pt2, color, thickness);
}
