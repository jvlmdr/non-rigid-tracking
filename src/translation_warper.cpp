#include "translation_warp.hpp"

TranslationWarper::TranslationWarper(const CostFunction& function)
    : function_(&function) {}

TranslationWarper::TranslationWarper() : function_(NULL) {}

TranslationWarper::~TranslationWarper() {}

int TranslationWarper::numParams() const {
  return NUM_PARAMS;
}

cv::Point2d TranslationWarper::evaluate(const cv::Point2d& position,
                                        const double* params,
                                        double* jacobian) const {
  double x[2] = { position.x, position.y };
  const double* param_blocks[2] = { x, params };

  double y[2];
  double* jacobian_blocks[2] = { NULL, jacobian };

  // Use Ceres to get the derivative of the warp function.
  function_->Evaluate(param_blocks, y, jacobian_blocks);

  return cv::Point2d(y[0], y[1]);
}

cv::Mat TranslationWarper::matrix(const double* params) const {
  double x = params[0];
  double y = params[1];

  cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, x, 0, 1, y);

  return M;
}

bool TranslationWarper::isValid(const double* params,
                                const cv::Size& image_size,
                                int radius) const {
  double x = params[0];
  double y = params[1];

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
