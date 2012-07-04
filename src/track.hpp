#ifndef TRACK_HPP_
#define TRACK_HPP_

#include <opencv2/core/core.hpp>
#include <map>

// A track is a set of (t, x) pairs.
typedef std::map<int, cv::Point2d> Track;

#endif
