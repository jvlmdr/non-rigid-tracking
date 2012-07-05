#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include "track.hpp"

// Describes a collection of tracks from a single video sequence.
typedef std::vector<Track> TrackList;

bool saveTracks(const std::string& filename,
                const cv::Size& size,
                const TrackList& tracks);

// Generates a set of point tracks from an image sequence.
class Tracker {
  public:
    // Pure virtual.
    virtual ~Tracker() = 0;

    // Feeds in the next image.
    virtual void feed(const cv::Mat& image) = 0;

    // Returns number of images so far.
    virtual int numFrames() const = 0;
    // Provides read access to all tracks.
    // Reference should be valid until non-const function called.
    virtual const TrackList& tracks() const = 0;
};

////////////////////////////////////////////////////////////////////////////////

// Tracker which writes out data after each frame.
class SerialTracker : public Tracker {
  public:
    // Pure virtual.
    virtual ~SerialTracker() = 0;

    // Writes current frame to file storage.
    virtual bool write(cv::FileStorage& out) const = 0;
    // Outputs a list of tracks which are currently being tracked.
    virtual void activeTracks(std::vector<const Track*>& tracks) const = 0;
};

#endif
