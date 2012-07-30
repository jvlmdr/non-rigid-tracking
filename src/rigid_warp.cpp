#include "rigid_warp.hpp"
#include <cmath>
#include "rigid_feature.hpp"

const int LINE_THICKNESS = 2;

////////////////////////////////////////////////////////////////////////////////

RigidWarpFunction::RigidWarpFunction(double resolution)
    : resolution_(resolution) {}

////////////////////////////////////////////////////////////////////////////////

RigidWarp::RigidWarp(int resolution)
    : resolution_(resolution), warp_(new RigidWarpFunction(resolution)) {}

RigidWarp::~RigidWarp() {}

cv::Point2d RigidWarp::evaluate(const cv::Point2d& position,
                                const double* params,
                                double* jacobian) const {
  // Get derivative of warp function.
  double x[2] = { position.x, position.y };
  // Create somewhere to write out the result.
  double y[2];

  const double* param_blocks[2] = { x, params };
  double* jacobian_blocks[2] = { NULL, jacobian };

  // Use Ceres to get the derivative of the warp function.
  warp_.Evaluate(param_blocks, y, jacobian_blocks);

  return cv::Point2d(y[0], y[1]);
}

int RigidWarp::numParams() const {
  return NUM_PARAMS;
}

cv::Mat RigidWarp::matrix(const double* params) const {
  const RigidFeature* p = reinterpret_cast<const RigidFeature*>(params);

  double scale = p->size / double(resolution_);
  cv::Mat M = (cv::Mat_<double>(2, 3) <<
      scale * std::cos(p->theta), scale * -std::sin(p->theta), p->x,
      scale * std::sin(p->theta), scale *  std::cos(p->theta), p->y);

  return M;
}

void RigidWarp::draw(cv::Mat& image,
                     const double* params,
                     int width,
                     const cv::Scalar& color) const {
  const RigidFeature* p = reinterpret_cast<const RigidFeature*>(params);

  double radius = width / 2.;
  cv::Point2d c(0, 0);
  cv::Point2d i(radius, 0);
  cv::Point2d j(0, radius);
  c = evaluate(c, params, NULL);
  i = evaluate(i, params, NULL);
  j = evaluate(j, params, NULL);
  i -= c;
  j -= c;

  cv::line(image, c - i - j, c + i - j, color, LINE_THICKNESS);
  cv::line(image, c - i + j, c + i + j, color, LINE_THICKNESS);
  cv::line(image, c - i - j, c - i + j, color, LINE_THICKNESS);
  cv::line(image, c + i - j, c + i + j, color, LINE_THICKNESS);
  cv::line(image, c, c + j, color, LINE_THICKNESS);
}
