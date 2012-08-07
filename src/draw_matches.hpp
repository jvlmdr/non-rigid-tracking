#ifndef DRAW_MATCHES_HPP_
#define DRAW_MATCHES_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include "match.hpp"
#include "rigid_feature.hpp"

void drawMatches(const std::vector<RigidFeature>& keypoints1,
                 const std::vector<RigidFeature>& keypoints2,
                 const std::vector<Match>& matches,
                 cv::Mat& image1,
                 cv::Mat& image2,
                 cv::Mat& render);

#endif
