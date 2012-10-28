#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <opencv2/core/core.hpp>

cv::Point2d project(const cv::Mat& P, const cv::Point3d& x);
cv::Matx33d computeFundMatFromCameras(const cv::Matx34d& P1,
                                      const cv::Matx34d& P2);

#endif
