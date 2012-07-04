#ifndef READ_IMAGE_HPP_
#define READ_IMAGE_HPP_

#include <string>
#include <opencv2/core/core.hpp>

bool readImage(const std::string& filename, cv::Mat& color, cv::Mat& gray);

#endif
