#include "translation_feature_drawer.hpp"

TranslationFeatureDrawer::TranslationFeatureDrawer(const cv::Point2d& point,
                                                   int radius)
    : point_(point), radius_(radius) {}

TranslationFeatureDrawer::~TranslationFeatureDrawer() {}

void TranslationFeatureDrawer::draw(cv::Mat& image,
                                    const cv::Scalar& color,
                                    int thickness) const {
  cv::Point2d pt1 = point_ - cv::Point2d(radius_ + 1, radius_ + 1);
  cv::Point2d pt2 = point_ + cv::Point2d(radius_ + 1, radius_ + 1);
  cv::rectangle(image, pt1, pt2, color, thickness);
}
