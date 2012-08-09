#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <opencv2/core/core.hpp>

cv::Point2d project(const cv::Mat& P, const cv::Point3d& x);
cv::Mat computeFundMatFromCameras(const cv::Mat& P1, const cv::Mat& P2);

#endif
