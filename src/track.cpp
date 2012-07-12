#include "track.hpp"

TrackCursor::TrackCursor() : track(NULL), point() {}

TrackCursor::TrackCursor(const Track& track, Track::const_iterator point)
    : track(&track), point(point) {}

TrackCursor TrackCursor::make(const Track& track) {
  return TrackCursor(track, track.begin());
}

bool TrackCursor::end() const {
  return point == track->end();
}
