#ifndef KLT_TRACKER_HPP_
#define KLT_TRACKER_HPP_

#include <opencv2/core/core.hpp>
#include <vector>
#include <list>
#include "klt.h"

#include "tracker.hpp"

class KltTracker : public SerialTracker {
  public:
    KltTracker();
    void init(int max_num_features,
              int min_clearance,
              int window_size,
              int min_eigenvalue,
              int cornerness_jump,
              double min_determinant,
              int max_iterations,
              double min_displacement,
              double max_residual,
              int consistency_mode);

    // Processes the next frame.
    void feed(const cv::Mat& image);

    // Returns the number of frames (so far).
    int numFrames() const;
    // Accesses the tracks.
    const TrackList& tracks() const;

    // Writes current frame to file storage.
    bool write(cv::FileStorage& out) const;
    // Returns the tracks which are currently active.
    void activeTracks(std::vector<const Track*>& tracks) const;

  private:
    // All tracks.
    TrackList tracks_;
    // Active subset of tracks.
    typedef std::list<int> Subset;
    Subset active_;
    // Current frame index.
    int frame_number_;
    // Previous image.
    cv::Mat previous_;

    KLT_TrackingContext tc_;
    KLT_FeatureList fl_;
};

#endif
