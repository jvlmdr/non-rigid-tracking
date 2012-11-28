#ifndef DRAW_SCALE_SPACE_POSITION_HPP_
#define DRAW_SCALE_SPACE_POSITION_HPP_

#include <opencv2/core/core.hpp>
#include "scale_space_position.hpp"

void drawScaleSpacePosition(cv::Mat& image,
                            const ScaleSpacePosition& feature,
                            int radius,
                            const cv::Scalar& color,
                            int thickness);

#endif
