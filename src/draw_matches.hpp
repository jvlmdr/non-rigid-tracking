#ifndef DRAW_MATCHES_HPP_
#define DRAW_MATCHES_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "match.hpp"

void drawMatches(const std::vector<cv::KeyPoint>& keypoints1,
                 const std::vector<cv::KeyPoint>& keypoints2,
                 const MatchList& matches,
                 cv::Mat& image1,
                 cv::Mat& image2,
                 cv::Mat& render);

#endif
