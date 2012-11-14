#ifndef DRAW_MATCHES_HPP_
#define DRAW_MATCHES_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include "match.hpp"
#include "sift_position.hpp"

void drawMatches(const std::vector<SiftPosition>& keypoints1,
                 const std::vector<SiftPosition>& keypoints2,
                 const std::vector<Match>& matches,
                 const cv::Mat& image1,
                 const cv::Mat& image2,
                 cv::Mat& render,
                 int line_thickness);

#endif
