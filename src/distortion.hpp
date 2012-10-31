#ifndef DISTORTION_HPP_
#define DISTORTION_HPP_

#include <opencv2/core/core.hpp>

cv::Point2d undistort(const cv::Point2d& x, double w);
cv::Point2d distort(const cv::Point2d& x, double w);

double distortRadius(double x, double w);
double undistortRadius(double y, double w);
double maxDistortedRadius(double w);

#endif
