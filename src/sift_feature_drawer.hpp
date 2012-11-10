#ifndef SIFT_FEATURE_DRAWER_HPP_
#define SIFT_FEATURE_DRAWER_HPP_

#include "feature_drawer.hpp"
#include "sift_position.hpp"

class SiftFeatureDrawer : public FeatureDrawer {
  public:
    SiftFeatureDrawer(const SiftPosition& position);
    ~SiftFeatureDrawer();
    void draw(cv::Mat& image, const cv::Scalar& color, int thickness) const;

  private:
    SiftPosition position_;
};

#endif
