#ifndef TRACK_HPP_
#define TRACK_HPP_

#include <opencv2/core/core.hpp>
#include <map>

// A track is a set of (t, x) pairs.
typedef std::map<int, cv::Point2d> Track;

// A TrackCursor is an iterator that can check if it has reached the end.
// Don't C++ iterators have the ability to do this? Not for lists?
struct TrackCursor {
  const Track* track;
  Track::const_iterator point;

  TrackCursor();
  TrackCursor(const Track& track, Track::const_iterator point);

  bool end() const;

  // Returns a cursor at the start of the track.
  static TrackCursor make(const Track& track);
};

#endif
