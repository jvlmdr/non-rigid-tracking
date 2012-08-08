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
#include "match_reader.hpp"
#include "vector_reader.hpp"

typedef std::vector<cv::KeyPoint> KeypointList;

// Save keypoint index not whole descriptor.
// OpenCV always reads the whole file at once.
// That would be a lot of descriptors.
struct IndexedPoint {
  int index;
  cv::Point2d point;

  IndexedPoint(int index, const cv::Point2d& point)
      : index(index), point(point) {}

  // Default constructor.
  IndexedPoint() : index(-1), point() {}
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

  typedef std::map<int, int> Lookup;
  Lookup previous_active_tracks;
  KeypointList previous_keypoints;

  while (ok) {
    // Load keypoints.
    KeypointList keypoints;
    std::string keypoints_file = makeFilename(keypoints_format, u);
    ok = loadKeypoints(keypoints_file, keypoints);
    if (!ok) {
      std::cerr << "could not load keypoints" << std::endl;
      continue;
    }

    int t = u - 1;

    if (t >= 0) {
      std::cout << "(" << t << ", " << u << ")" << std::endl;

      // Load matches.
      typedef std::vector<Match> MatchList;
      MatchList matches;
      std::string matches_file = makeFilename(matches_format, t);

      MatchReader match_reader;
      VectorReader<Match> match_list_reader(match_reader);
      ok = load(matches_file, matches, match_list_reader);
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
          tracks[index][u] = IndexedPoint(b, keypoints[b].pt);
          active_tracks[b] = index;
        } else {
          // It wasn't: create a new track.
          tracks.push_back(Track_<IndexedPoint>());
          tracks.back()[t] = IndexedPoint(a, previous_keypoints[a].pt);
          tracks.back()[u] = IndexedPoint(b, keypoints[b].pt);
          active_tracks[b] = tracks.size() - 1;
        }
      }

      previous_active_tracks.swap(active_tracks);
    }

    previous_keypoints.swap(keypoints);
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
