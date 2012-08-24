#ifndef DRAW_MATCHES_HPP_
#define DRAW_MATCHES_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include "match.hpp"
#include "similarity_feature.hpp"

void drawMatches(const std::vector<SimilarityFeature>& keypoints1,
                 const std::vector<SimilarityFeature>& keypoints2,
                 const std::vector<Match>& matches,
                 cv::Mat& image1,
                 cv::Mat& image2,
                 cv::Mat& render);

#endif
