#ifndef SCALE_SPACE_FEATURE_DRAWER_HPP_
#define SCALE_SPACE_FEATURE_DRAWER_HPP_

#include "feature_drawer.hpp"
#include "scale_space_position.hpp"

class ScaleSpaceFeatureDrawer : public FeatureDrawer {
  public:
    ScaleSpaceFeatureDrawer(const ScaleSpacePosition& position, int radius);
    ~ScaleSpaceFeatureDrawer();
    void draw(cv::Mat& image, const cv::Scalar& color, int thickness) const;

  private:
    ScaleSpacePosition position_;
    int radius_;
};

#endif
