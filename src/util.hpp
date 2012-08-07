#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <opencv2/core/core.hpp>

inline double sqr(double x);

inline cv::Point2d convertPointFromHomogeneous2(const cv::Mat& X);

#include "util.inl"

#endif
