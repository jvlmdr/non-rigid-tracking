#include "tracking/translation-warp.hpp"

namespace tracking {

TranslationWarp::TranslationWarp(double x, double y)
    : params_(NUM_PARAMS, 0.) {
  params_[0] = x;
  params_[1] = y;
}

TranslationWarp::TranslationWarp() : params_(NUM_PARAMS, 0.) {}

TranslationWarp::~TranslationWarp() {}

int TranslationWarp::numParams() const {
  return NUM_PARAMS;
}

cv::Point2d TranslationWarp::evaluate(const cv::Point2d& position,
                                      double* jacobian) const {
  TranslationWarper warper;
  return warper.evaluate(position, params(), jacobian);
}

cv::Mat TranslationWarp::matrix() const {
  TranslationWarper warper;
  return warper.matrix(params());
}

bool TranslationWarp::isValid(const cv::Size& image_size, int radius) const {
  TranslationWarper warper;
  return warper.isValid(params(), image_size, radius);
}

void TranslationWarp::draw(cv::Mat& image,
                           int radius,
                           const cv::Scalar& color,
                           int thickness) const {
  double x = params_[0];
  double y = params_[1];

  cv::Point center(std::floor(x + 0.5), std::floor(y + 0.5));
  cv::Point offset(radius, radius);

  cv::Point pt1 = center - offset;
  cv::Point pt2 = center + offset;

  cv::rectangle(image, pt1, pt2, color, thickness);
}

double* TranslationWarp::params() {
  return &params_.front();
}

const double* TranslationWarp::params() const {
  return &params_.front();
}

Warper* TranslationWarp::newWarper() const {
  return new TranslationWarper();
}

} // namespace tracking
