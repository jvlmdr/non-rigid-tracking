#include "tracking/similarity-warp.hpp"
#include <cmath>

namespace tracking {

SimilarityWarp::SimilarityWarp(double x,
                               double y,
                               double log_scale,
                               double theta)
    : params_(NUM_PARAMS, 0.) {
  params_[0] = x;
  params_[1] = y;
  params_[2] = log_scale;
  params_[3] = theta;
}

SimilarityWarp::SimilarityWarp() : params_(NUM_PARAMS, 0.) {}

SimilarityWarp::~SimilarityWarp() {}

int SimilarityWarp::numParams() const {
  return NUM_PARAMS;
}

cv::Point2d SimilarityWarp::evaluate(const cv::Point2d& position,
                                     double* jacobian) const {
  SimilarityWarper warper;
  return warper.evaluate(position, params(), jacobian);
}

cv::Mat SimilarityWarp::matrix() const {
  SimilarityWarper warper;
  return warper.matrix(params());
}

bool SimilarityWarp::isValid(const cv::Size& image_size,
                             int patch_radius) const {
  SimilarityWarper warper;
  return warper.isValid(params(), image_size, patch_radius);
}

void SimilarityWarp::draw(cv::Mat& image,
                          int radius,
                          const cv::Scalar& color,
                          int thickness) const {
  double x = params_[0];
  double y = params_[1];
  double log_scale = params_[2];
  double theta = params_[3];
  double scale = std::exp(log_scale);
  radius = scale * radius;

  cv::Point2d center(x, y);
  cv::Point2d line = radius * cv::Point2d(std::cos(theta), std::sin(theta));

  cv::circle(image, center, radius, color, thickness);
  cv::line(image, center, center + line, color, thickness);
}

double* SimilarityWarp::params() {
  return &params_.front();
}

const double* SimilarityWarp::params() const {
  return &params_.front();
}

Warper* SimilarityWarp::newWarper() const {
  return new SimilarityWarper();
}

} // namespace tracking
