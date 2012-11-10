#ifndef FEATURE_DRAWER_HPP_
#define FEATURE_DRAWER_HPP_

#include <opencv2/core/core.hpp>

class FeatureDrawer {
  public:
    virtual ~FeatureDrawer() {}
    virtual void draw(cv::Mat& image,
                      const cv::Scalar& color,
                      int thickness) const = 0;
};

#endif
