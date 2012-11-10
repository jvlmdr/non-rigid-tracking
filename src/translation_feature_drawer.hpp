#ifndef TRANSLATION_FEATURE_DRAWER_HPP_
#define TRANSLATION_FEATURE_DRAWER_HPP_

#include "feature_drawer.hpp"

class TranslationFeatureDrawer : public FeatureDrawer {
  public:
    TranslationFeatureDrawer(const cv::Point2d& point, int radius);
    ~TranslationFeatureDrawer();
    void draw(cv::Mat& image, const cv::Scalar& color, int thickness) const;

  private:
    cv::Point2d point_;
    int radius_;
};

#endif
