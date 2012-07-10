#ifndef RANDOM_COLOR_HPP_
#define RANDOM_COLOR_HPP_

#include <opencv2/core/core.hpp>

// Returns a random color with given saturation and brightness.
cv::Scalar randomColor(double saturation, double brightness);

#endif
