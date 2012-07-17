#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include "read_image.hpp"
#include "track_list.hpp"
#include "keypoint.hpp"
#include "match.hpp"

// Save keypoint index not whole descriptor.
// OpenCV always reads the whole file at once.
// That would be a lot of descriptors.

struct IndexedPoint {
  cv::Point2d point;
  int index;

  IndexedPoint(const cv::Point2d& point, int index)
      : point(point), index(index) {}

  // Default constructor.
  IndexedPoint() : point(), index() {}
};

struct WriteIndexedPoint : public Write<IndexedPoint> {
  ~WriteIndexedPoint() {}

  void operator()(cv::FileStorage& file, const IndexedPoint& x) {
    file << "{:";
    file << "x" << x.point.x;
    file << "y" << x.point.y;
    file << "index" << x.index;
    file << "}";
  }
};

struct ReadIndexedPoint : public Read<IndexedPoint> {
  ~ReadIndexedPoint() {}

  void operator()(const cv::FileNode& node, IndexedPoint& x) {
    x.point.x = (double)node["x"];
    x.point.y = (double)node["y"];
    x.index = (int)node["index"];
  }
};

// Constructs indexed points from an unindexed list.
struct MakeIndexedPoint {
  int i;

  MakeIndexedPoint() : i(0) {}

  IndexedPoint operator()(const cv::KeyPoint& keypoint) {
    IndexedPoint feature(keypoint.pt, i);
    i += 1;
    return feature;
  }
};

typedef std::map<int, int> Lookup;
typedef std::vector<IndexedPoint> PointList;
typedef std::vector<cv::KeyPoint> KeypointList;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Creates tracks from matches between adjacent frames." << std::endl;
  usage << std::endl;
  usage << argv[0] << " matches-format keypoints-format tracks" << std::endl;
  usage << std::endl;
  usage << "matches-format -- Input. Matches between frame t and frame"
    " t + 1." << std::endl;
  usage << "keypoints-format -- Input. Keypoints in frame t." << std::endl;
  usage << "tracks -- Output. Tracks through time." << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string matches_format = argv[1];
  std::string keypoints_format = argv[2];
  std::string tracks_file = argv[3];

  TrackList_<IndexedPoint> tracks;

  bool ok = true;
  // Examine frames (u - 1, u).
  int u = 0;

  Lookup previous_active_tracks;
  PointList previous_features;

  while (ok) {
    // Load keypoints.
    KeypointList keypoints;
    std::string keypoints_file = makeFilename(keypoints_format, u);
    ok = loadKeypoints(keypoints_file, keypoints);
    if (!ok) {
      std::cerr << "could not load keypoints" << std::endl;
      continue;
    }

    // Build features.
    PointList features;
    std::transform(keypoints.begin(), keypoints.end(),
        std::back_inserter(features), MakeIndexedPoint());

    int t = u - 1;

    if (t >= 0) {
      std::cout << "(" << t << ", " << u << ")" << std::endl;

      // Load matches.
      MatchList matches;
      std::string matches_file = makeFilename(matches_format, t);
      ok = loadMatches(matches_file, matches);
      if (!ok) {
        continue;
      }

      Lookup active_tracks;

      // Iterate through the matches.
      MatchList::const_iterator match;
      for (match = matches.begin(); match != matches.end(); ++match) {
        // Match associates keypoints (a, b).
        int a = match->first;
        int b = match->second;

        // Find whether keypoint was matched in the previous frame.
        Lookup::const_iterator previous_track = previous_active_tracks.find(a);
        if (previous_track != previous_active_tracks.end()) {
          // It was: extend the track.
          int index = previous_track->second;
          tracks[index][u] = features[b];
          active_tracks[b] = index;
        } else {
          // It wasn't: create a new track.
          tracks.push_back(Track_<IndexedPoint>());
          tracks.back()[t] = previous_features[a];
          tracks.back()[u] = features[b];
          active_tracks[b] = tracks.size() - 1;
        }
      }

      previous_active_tracks.swap(active_tracks);
    }

    previous_features.swap(features);
    u += 1;
  }

  // Write out matches.
  WriteIndexedPoint write;
  ok = tracks.save(tracks_file, write);
  if (!ok) {
    std::cerr << "could not save tracks to file" << std::endl;
    return 1;
  }

  return 0;
}
