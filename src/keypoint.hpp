#ifndef KEYPOINT_HPP_
#define KEYPOINT_HPP_

#include <string>
#include <vector>
#include <opencv2/features2d/features2d.hpp>

bool saveKeypoints(const std::string& filename,
                   const std::vector<cv::KeyPoint>& keypoints);

bool loadKeypoints(const std::string& filename,
                   std::vector<cv::KeyPoint>& keypoints);

#endif
