#ifndef HSV_HPP_
#define HSV_HPP_

#include <opencv2/core/core.hpp>

cv::Scalar hsvToRgb(double h, double s, double v);

void evenlySpacedColors(int n,
                        double s,
                        double v,
                        std::vector<cv::Scalar>& colors);

#endif
