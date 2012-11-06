#ifndef ADMM_TRACKING_HPP_
#define ADMM_TRACKING_HPP_

#include <vector>
#include <opencv2/core/core.hpp>

double findClassifierTrackAdmm(const std::vector<cv::Mat>& responses,
                               std::vector<cv::Point2d>& positions,
                               double lambda,
                               double rho);

#endif
