#ifndef TRACK_LIST_HPP_
#define TRACK_LIST_HPP_

#include <vector>
#include <list>
#include <map>
#include <string>
#include <opencv2/core/core.hpp>
#include "track.hpp"

// Describes a collection of tracks from a single video sequence.
typedef std::vector<Track> TrackList;

// Saves a list of tracks to a file.
bool saveTracks(const std::string& filename,
                const cv::Size& size,
                const TrackList& tracks);

// Loads a list of tracks from a file.
bool loadTracks(const std::string& filename,
                cv::Size& size,
                TrackList& tracks,
                int* num_tracks);

// Returns the first frame in any track.
int findFirstFrame(const TrackList& tracks);

// Describes the points present in a frame, indexed by track number.
typedef std::map<int, cv::Point2d> IndexedPoints;
typedef IndexedPoints::value_type IndexedPoint;

// Iterates through a list of tracks one frame at a time.
class FrameIterator {
  public:
    // Initializes at start of tracks.
    FrameIterator(const TrackList& tracks);
    // Copy constructor.
    FrameIterator(const FrameIterator& rhs);

    // Pre-increment.
    FrameIterator& operator++();
    // Post-increment.
    FrameIterator operator++(int);

    // Writes out the points in the current frame.
    void getPoints(IndexedPoints& points) const;
    // Returns true if this is the last frame.
    bool end() const;

  private:
    // Maintain a list of positions in each track.
    typedef std::list<TrackCursor> CursorList;
    CursorList cursors_;
    // Index of current frame.
    int t_;

    // Updates the list of points in this frame and advances the cursor.
    void advance();
};

#endif
