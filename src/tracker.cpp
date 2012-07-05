#include "tracker.hpp"
#include <iostream>
#include <string>
#include <map>

Tracker::~Tracker() {}

SerialTracker::~SerialTracker() {}

// A TrackCursor is an iterator that can check if it has reached the end.
// Don't C++ iterators have the ability to do this? Not for lists?
struct TrackCursor {
  const Track* track;
  Track::const_iterator point;

  TrackCursor(const Track& track, Track::const_iterator point)
      : track(&track), point(point) {}

  TrackCursor() : track(NULL), point() {}
};

TrackCursor cursorToStartOfTrack(const Track& track) {
  return TrackCursor(track, track.begin());
}

bool saveTracks(const std::string& filename,
                const cv::Size& size,
                const TrackList& tracks) {
  // Open file to save tracks.
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    std::cerr << "could not open file " << filename << std::endl;
    return false;
  }

  file << "width" << size.width;
  file << "height" << size.height;

  // Give every track an index.
  typedef std::map<int, TrackCursor> Map;
  Map map;
  {
    TrackList::const_iterator track;
    int i = 0;
    for (track = tracks.begin(); track != tracks.end(); ++track) {
      map[i] = cursorToStartOfTrack(*track);
      i += 1;
    }
  }

  // Save each frame out to file.
  file << "tracks" << "[";

  int t = 0;

  while (!map.empty()) {
    file << "[";

    Map::iterator feature = map.begin();
    while (feature != map.end()) {
      // Unpack.
      TrackCursor& cursor = feature->second;
      // Whether feature should be removed at the end.
      bool remove;

      if (cursor.point->first != t) {
        remove = false;
      } else {
        // Unpack more.
        int id = feature->first;
        const cv::Point2d& w = cursor.point->second;

        // Feature observed in this frame.
        file << "{:";
        file << "id" << id;
        file << "x" << w.x;
        file << "y" << w.y;
        file << "}";

        // Advance cursor.
        ++cursor.point;
        if (cursor.point == cursor.track->end()) {
          remove = true;
        } else {
          remove = false;
        }
      }

      if (remove) {
        map.erase(feature++);
      } else {
        ++feature;
      }
    }

    file << "]";
    t += 1;
  }

  file << "]";

  return true;
}
