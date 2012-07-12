#include "track_list.hpp"
#include <iostream>
#include <map>
#include <numeric>
#include <limits>

////////////////////////////////////////////////////////////////////////////////
// FrameIterator

// Initializes at start of tracks.
FrameIterator::FrameIterator(const TrackList& tracks) : cursors_(), t_(0) {
  // Iterate through tracks.
  TrackList::const_iterator track = tracks.begin();
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Add a cursor for each non-empty track.
    if (!track->empty()) {
      cursors_.push_back(TrackCursor::make(*track));
    }
  }
}

// Copy constructor.
FrameIterator::FrameIterator(const FrameIterator& rhs)
    : cursors_(rhs.cursors_), t_(rhs.t_) {}

// Pre-increment.
// Updates the list of points in this frame and advances the cursor.
FrameIterator& FrameIterator::operator++() {
  // Iterate through tracks.
  CursorList::iterator cursor = cursors_.begin();
  while (cursor != cursors_.end()) {
    bool remove = false;

    // Get space-time point.
    Track::const_iterator space_time = cursor->point;
    int t = space_time->first;

    // If track appeared in this frame, advance the cursor.
    if (t == t_) {
      ++cursor->point;
      // If the cursor reached the end of the track, remove it.
      if (cursor->end()) {
        remove = true;
      }
    }

    if (remove) {
      cursor = cursors_.erase(cursor);
    } else {
      ++cursor;
    }
  }

  t_ += 1;

  return *this;
}

// Post-increment.
FrameIterator FrameIterator::operator++(int) {
  FrameIterator result(*this);
  ++(*this);
  return result;
}

void FrameIterator::getPoints(IndexedPoints& points) const {
  // Iterate through tracks.
  int i = 0;
  CursorList::const_iterator cursor;

  for (cursor = cursors_.begin(); cursor != cursors_.end(); ++cursor) {
    // Get space-time point.
    Track::const_iterator space_time = cursor->point;
    int t = space_time->first;

    // If track appeared in this frame, add to points.
    if (t == t_) {
      const cv::Point2d& x = space_time->second;
      points[i] = x;
    }

    i += 1;
  }
}

// Returns true if this is the last frame.
bool FrameIterator::end() const {
  return cursors_.empty();
}

////////////////////////////////////////////////////////////////////////////////

namespace {

int firstFrame(const Track& track) {
  return track.begin()->first;
}

}

template<class T>
T min(const T& a, const T& b) {
  return std::min(a, b);
}

// Returns the first frame in any track.
int findFirstFrame(const TrackList& tracks) {
  std::vector<int> indices(tracks.size());
  std::transform(tracks.begin(), tracks.end(), indices.begin(), firstFrame);

  return std::accumulate(indices.begin(), indices.end(),
      std::numeric_limits<int>::max(), min<int>);
}

// Saves a list of tracks to a file.
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
      map[i] = TrackCursor::make(*track);
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

// Loads a list of tracks from a file.
bool loadTracks(const std::string& filename,
                cv::Size& size,
                TrackList& tracks,
                int* num_frames) {
  // Open file to read tracks.
  cv::FileStorage file(filename, cv::FileStorage::READ);
  if (!file.isOpened()) {
    std::cerr << "could not open file " << filename << std::endl;
    return false;
  }

  // Get image dimensions.
  int width = (int)file["width"];
  int height = (int)file["height"];
  if (width <= 0 || height <= 0) {
    std::cerr << "could not read width or height" << std::endl;
    return false;
  }

  size = cv::Size(width, height);

  // Map from integer IDs to track indices.
  typedef std::map<int, int> Lookup;
  Lookup lookup;

  cv::FileNode frames = file["tracks"];
  cv::FileNodeIterator frame;
  int t = 0;

  for (frame = frames.begin(); frame != frames.end(); ++frame) {
    cv::FileNode points = (*frame);

    cv::FileNodeIterator point;
    for (point = points.begin(); point != points.end(); ++point) {
      // Read point data from file.
      int id = (int)((*point)["id"]);
      double x = (double)((*point)["x"]);
      double y = (double)((*point)["y"]);
      cv::Point2d p(x, y);

      // Check whether track exists.
      Lookup::iterator e = lookup.find(id);
      Track* track;

      if (e == lookup.end()) {
        // Does not exist. Create new track.
        tracks.push_back(Track());
        track = &tracks.back();
        lookup[id] = tracks.size() - 1;
      } else {
        // Add to existing track.
        track = &tracks[e->second];
      }

      // Add the point at this time instant.
      (*track)[t] = p;
    }

    t += 1;
  }

  if (num_frames != NULL) {
    *num_frames = t;
  }

  return true;
}
