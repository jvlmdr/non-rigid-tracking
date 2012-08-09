#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <opencv2/core/core.hpp>

inline double sqr(double x);

// Differentiate between 2D and 3D points as "image" and "world" points.

inline cv::Mat imagePointToHomogeneous(const cv::Point2d& x);
inline cv::Point2d imagePointFromHomogeneous(const cv::Mat& X);

inline cv::Mat worldPointToHomogeneous(const cv::Point3d& x);
inline cv::Point3d worldPointFromHomogeneous(const cv::Mat& X);

#include "util.inl"

#endif
