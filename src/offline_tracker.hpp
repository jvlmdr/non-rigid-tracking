#ifndef OFFLINE_TRACKER_HPP_
#define OFFLINE_TRACKER_HPP_

#include <opencv2/core/core.hpp>
#include "video.hpp"
#include "space_time_image_point.hpp"
#include "track.hpp"

// Tracks points through a video sequence.
class OfflineTracker {
  public:
    virtual ~OfflineTracker() {}
    virtual void init(const Video& video) = 0;
    virtual bool track(const SpaceTimeImagePoint& point,
                       Track<cv::Point2d>& track) const = 0;
};

#endif
