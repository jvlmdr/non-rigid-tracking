#ifndef VIDEOSEG_REGION_HPP_
#define VIDEOSEG_REGION_HPP_

#include "videoseg/using.hpp"
#include "videoseg/region.pb.h"
#include <opencv2/core/core.hpp>

namespace videoseg {

bool regionContains(const Rasterization& region, cv::Point pos);

// Merge many 2D regions.
void mergeRegions(const vector<const Rasterization*>& regions,
                  Rasterization& result);

}

#endif
