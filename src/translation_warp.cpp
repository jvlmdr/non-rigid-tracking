#include "translation_warp.hpp"

TranslationWarp(double x, double y, const TranslationWarper& warper)
    : params_(NUM_PARAMS, 0.), warper_(&warper) {
  params_[0] = x;
  params_[1] = y;
}

TranslationWarp::TranslationWarp() : params_(NUM_PARAMS, 0.), warp_(NULL) {}

TranslationWarp::~TranslationWarp() {}

int TranslationWarp::numParams() const {
  return warper_->numParams();
}

cv::Point2d TranslationWarp::evaluate(const cv::Point2d& position,
                                      double* jacobian) const {
  return warper_->evaluate(position, params(), jacobian);
}

cv::Mat TranslationWarp::matrix() const {
  return warper_->matrix(params());
}

bool TranslationWarp::isValid(const cv::Size& image_size, int radius) const {
  return warper_->isValid(params(), image_size, radius);
}

double* TranslationWarp::params() {
  return &params_.front();
}

const double* TranslationWarp::params() const {
  return &params_.front();
}

const Warper* TranslationWarp::warper() const {
  return warper_;
}
