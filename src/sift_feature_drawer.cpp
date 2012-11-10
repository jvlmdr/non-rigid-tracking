#include "sift_feature_drawer.hpp"
#include "draw_sift_position.hpp"

SiftFeatureDrawer::SiftFeatureDrawer(const SiftPosition& position)
    : position_(position) {}

SiftFeatureDrawer::~SiftFeatureDrawer() {}

void SiftFeatureDrawer::draw(cv::Mat& image,
                             const cv::Scalar& color,
                             int thickness) const {
  drawSiftPosition(image, position_, color, thickness);
}
