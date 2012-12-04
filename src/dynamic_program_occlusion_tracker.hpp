#ifndef DYNAMIC_PROGRAM_OCCLUSION_TRACKER_HPP_
#define DYNAMIC_PROGRAM_OCCLUSION_TRACKER_HPP_

#include "offline_tracker.hpp"

class DynamicProgramOcclusionTracker : public OfflineTracker {
  public:
    // Parameters:
    // lambda -- Weighting of pairwise change relative to detector response.
    // penalty -- Paid for claiming an occlusion.
    // radius -- Tracked region will be (2 * radius + 1) x (2 * radius + 1).
    DynamicProgramOcclusionTracker(double lambda, double penalty, int radius);

    ~DynamicProgramOcclusionTracker();

    void init(const Video& video);
    bool track(const SpaceTimeImagePoint& point,
               Track<cv::Point2d>& track) const;

  private:
    const Video* video_;
    double lambda_;
    double penalty_;
    int radius_;
};

#endif
