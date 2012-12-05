#ifndef DYNAMIC_PROGRAM_TRACKER_HPP_
#define DYNAMIC_PROGRAM_TRACKER_HPP_

#include "offline_tracker.hpp"

class DynamicProgramTracker : public OfflineTracker {
  public:
    // Parameters:
    // lambda -- Weighting of pairwise change relative to detector response.
    // radius -- Tracked region will be (2 * radius + 1) x (2 * radius + 1).
    // fix_seed -- Should the point which seeded the track be constrained?
    DynamicProgramTracker(double lambda, int radius, bool fix_seed);
    ~DynamicProgramTracker();

    void init(const Video& video);
    bool track(const SpaceTimeImagePoint& point,
               Track<cv::Point2d>& track) const;

  private:
    const Video* video_;
    double lambda_;
    int radius_;
    bool fix_seed_;
};

#endif
