#include "rigid_warp.hpp"
#include <cmath>
#include "rigid_feature.hpp"

const int LINE_THICKNESS = 2;

RigidWarp::RigidWarp() : warp_(new RigidWarpFunction) {}

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

  cv::Mat M = (cv::Mat_<double>(2, 3) <<
      p->scale *  std::cos(p->theta), p->scale * std::sin(p->theta), p->x,
      p->scale * -std::sin(p->theta), p->scale * std::cos(p->theta), p->y);

  return M;
}

void RigidWarp::draw(cv::Mat& image,
                     const double* params,
                     int width,
                     const cv::Scalar& color) const {
  const RigidFeature* p = reinterpret_cast<const RigidFeature*>(params);

  cv::Point2d c(p->x, p->y);
  cv::Point2d i(std::cos(-p->theta), std::sin(-p->theta));
  cv::Point2d j(std::sin(-p->theta), -std::cos(-p->theta));

  double radius = p->scale * (width - 1) / 2;
  i *= radius;
  j *= radius;

  cv::line(image, c - i - j, c + i - j, color, LINE_THICKNESS);
  cv::line(image, c - i + j, c + i + j, color, LINE_THICKNESS);
  cv::line(image, c - i - j, c - i + j, color, LINE_THICKNESS);
  cv::line(image, c + i - j, c + i + j, color, LINE_THICKNESS);
  cv::line(image, c, c + j, color, LINE_THICKNESS);
}
