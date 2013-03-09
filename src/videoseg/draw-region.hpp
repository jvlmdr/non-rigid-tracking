#ifndef VIDEOSEG_DRAW_REGION_HPP_
#define VIDEOSEG_DRAW_REGION_HPP_

#include "videoseg/using.hpp"
#include "videoseg/region.hpp"
#include <opencv2/core/core.hpp>

namespace videoseg {

// Fill a region with a solid color.
void fillRegion(const Rasterization& region,
                cv::Mat& image,
                cv::Vec3b color);

// Draw a stroke around a region.
void drawRegionBoundary(const Rasterization& region,
                        cv::Mat& image,
                        cv::Vec3b color);

// Copy pixels from one image to another within a region.
void copyRegion(const Rasterization& region,
                const cv::Mat& src,
                cv::Mat& dst);

// Copy pixels from one image to another and blend with a color.
void copyAndBlendRegion(const Rasterization& region,
                        const cv::Mat& src,
                        cv::Vec3b& color,
                        double alpha,
                        cv::Mat& dst);

}

#endif
