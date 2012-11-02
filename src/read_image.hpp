#ifndef READ_IMAGE_HPP_
#define READ_IMAGE_HPP_

#include <string>
#include <opencv2/core/core.hpp>

bool readColorImage(const std::string& filename, cv::Mat& image);
bool readImage(const std::string& filename, cv::Mat& color, cv::Mat& gray);
bool readGrayImage(const std::string& filename, cv::Mat& image);

#endif
