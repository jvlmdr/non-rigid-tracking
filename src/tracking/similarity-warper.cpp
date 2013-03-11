#include "tracking/similarity-warper.hpp"
#include <cmath>
#include "util/is-finite.hpp"

namespace tracking {

namespace {

const double MIN_SCALE = 1.;

}

SimilarityWarper::SimilarityWarper()
    : function_(new SimilarityWarpFunction()) {}

SimilarityWarper::~SimilarityWarper() {}

int SimilarityWarper::numParams() const {
  return NUM_PARAMS;
}

cv::Point2d SimilarityWarper::evaluate(const cv::Point2d& position,
                                       const double* params,
                                       double* jacobian) const {
  double x[2] = { position.x, position.y };
  const double* param_blocks[2] = { x, params };

  double y[2];
  double* jacobian_blocks[2] = { NULL, jacobian };

  // Use Ceres to get the derivative of the warp function.
  function_.Evaluate(param_blocks, y, jacobian_blocks);

  return cv::Point2d(y[0], y[1]);
}

cv::Mat SimilarityWarper::matrix(const double* params) const {
  double x = params[0];
  double y = params[1];
  double log_scale = params[2];
  double theta = params[3];
  double scale = std::exp(log_scale);

  cv::Mat M = (cv::Mat_<double>(2, 3) <<
      scale * std::cos(theta), scale * -std::sin(theta), x,
      scale * std::sin(theta), scale *  std::cos(theta), y);

  return M;
}

bool SimilarityWarper::isValid(const double* params,
                               const cv::Size& image_size,
                               int patch_radius) const {
  double x = params[0];
  double y = params[1];
  double log_scale = params[2];
  double scale = std::exp(log_scale);

  if (!isFinite(scale)) {
    DLOG(INFO) << "Scale became infinite";
    return false;
  }

  if (scale < MIN_SCALE) {
    DLOG(INFO) << "Scale too small (" << scale << " < " << MIN_SCALE << ")";
    return false;
  }

  // Check if region is entirely within image.
  double radius = std::ceil(scale * patch_radius);

  // Reduce size by radius of patch.
  int width = image_size.width - 2. * radius;
  int height = image_size.height - 2. * radius;
  cv::Rect_<double> bounds(radius, radius, width, height);
  if (!bounds.contains(cv::Point2d(x, y))) {
    DLOG(INFO) << "Feature outside image";
    return false;
  }

  return true;
}

} // namespace tracking
