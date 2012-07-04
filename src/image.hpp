#ifndef IMAGE_HPP_
#define IMAGE_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

bool readNextFrame(cv::VideoCapture& capture, cv::Mat& color, cv::Mat& gray);

#endif
