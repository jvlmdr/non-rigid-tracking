#include "similarity_warp.hpp"
#include <cmath>
#include "util.hpp"

SimilarityWarp::SimilarityWarp(double x,
                               double y,
                               double log_scale,
                               double theta,
                               const SimilarityWarper& warper)
    : params_(NUM_PARAMS, 0.), warper_(&warper) {
  params_[0] = x;
  params_[1] = y;
  params_[2] = log_scale;
  params_[3] = theta;
}

SimilarityWarp::SimilarityWarp() : params_(NUM_PARAMS, 0.), warper_(NULL) {}

SimilarityWarp::~SimilarityWarp() {}

int SimilarityWarp::numParams() const {
  return warper_->numParams();
}

cv::Point2d SimilarityWarp::evaluate(const cv::Point2d& position,
                                     double* jacobian) const {
  return warper_->evaluate(position, params(), jacobian);
}

cv::Mat SimilarityWarp::matrix() const {
  return warper_->matrix(params());
}

bool SimilarityWarp::isValid(const cv::Size& image_size,
                             int patch_radius) const {
  return warper_->isValid(params(), image_size, patch_radius);
}

double* SimilarityWarp::params() {
  return &params_.front();
}

const double* SimilarityWarp::params() const {
  return &params_.front();
}

const Warper* SimilarityWarp::warper() const {
  return warper_;
}
