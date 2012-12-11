#ifndef DISTORTION_HPP_
#define DISTORTION_HPP_

#include <opencv2/core/core.hpp>

// Applies lens distortion to an undistorted point.
cv::Point2d distort(const cv::Point2d& x, double w);

// Removes lens distortion from a distorted, calibrated point.
// "Calibrated points" are related by the essential not the fundamental matrix.
cv::Point2d undistort(const cv::Point2d& x, double w);

// Returns true if a distorted, calibrated point can be undistorted.
// It cannot be undistorted if it falls outside the circle.
bool isUndistortable(const cv::Point2d& y, double w);

double distortRadius(double x, double w);
double undistortRadius(double y, double w);
double maxDistortedRadius(double w);

// Distort a point at infinity.
cv::Point2d distortPointAtInfinity(const cv::Point2d& x, double w);

#endif
