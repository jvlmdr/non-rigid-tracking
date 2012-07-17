#include "track_list.hpp"
#include <iostream>
#include <map>

WritePoint::~WritePoint() {}

void WritePoint::operator()(cv::FileStorage& file, const cv::Point2d& point) {
  file << "{:";
  file << "x" << point.x;
  file << "y" << point.y;
  file << "}";
}

ReadPoint::~ReadPoint() {}

void ReadPoint::operator()(const cv::FileNode& node, cv::Point2d& point) {
  point.x = (double)node["x"];
  point.y = (double)node["y"];
}

////////////////////////////////////////////////////////////////////////////////

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
