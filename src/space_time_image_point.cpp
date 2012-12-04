#include "space_time_image_point.hpp"

SpaceTimeImagePoint::SpaceTimeImagePoint() : p(0, 0), t(0) {}

SpaceTimeImagePoint::SpaceTimeImagePoint(const cv::Point2d& p, int t)
    : p(p), t(t) {}
