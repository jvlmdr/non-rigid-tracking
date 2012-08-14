#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include "track.hpp"
#include "track_list.hpp"

// Generates a set of point tracks from an image sequence.
class Tracker {
  public:
    // Pure virtual.
    virtual ~Tracker() {}

    // Feeds in the next image.
    virtual void feed(const cv::Mat& image) = 0;

    // Returns number of images so far.
    virtual int numFrames() const = 0;
    // Provides read access to all tracks.
    // Reference should be valid until non-const function called.
    virtual const TrackList<cv::Point2d>& tracks() const = 0;
};

////////////////////////////////////////////////////////////////////////////////

// Tracker which writes out data after each frame.
class SerialTracker : public Tracker {
  public:
    // Pure virtual.
    virtual ~SerialTracker() {}

    // Writes current frame to file storage.
    virtual bool write(cv::FileStorage& out) const = 0;
    // Outputs a list of tracks which are currently being tracked.
    virtual void activeTracks(
        std::vector<const Track<cv::Point2d>*>& tracks) const = 0;
};

#endif
