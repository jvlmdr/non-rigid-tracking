#ifndef KEYPOINT_HPP_
#define KEYPOINT_HPP_

#include <string>
#include <vector>
#include <opencv2/features2d/features2d.hpp>

bool saveKeyPoints(const std::string& filename,
                   const std::vector<cv::KeyPoint>& keypoints);

bool loadKeyPoints(const std::string& filename,
                   std::vector<cv::KeyPoint>& keypoints);

#endif
