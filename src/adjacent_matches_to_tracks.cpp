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
#include "sift_position.hpp"
#include "match.hpp"
#include "match_reader.hpp"
#include "vector_reader.hpp"
#include "sift_position_writer.hpp"
#include "track_list_writer.hpp"
#include "sift_position_reader.hpp"
#include "writer.hpp"

typedef std::vector<SiftPosition> KeypointList;

// Save index not whole descriptor.
// OpenCV always reads the whole file at once.
// That would be a lot of descriptors.
struct IndexedFeature {
  int index;
  SiftPosition feature;

  IndexedFeature(int index, const SiftPosition& feature)
      : index(index), feature(feature) {}

  // Default constructor.
  IndexedFeature() : index(-1), feature() {}
};

struct IndexedFeatureWriter : public Writer<IndexedFeature> {
  public:
    ~IndexedFeatureWriter() {}

    void write(cv::FileStorage& file, const IndexedFeature& x) {
      SiftPositionWriter feature_writer;
      file << "index" << x.index;
      feature_writer.write(file, x.feature);
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

  TrackList<IndexedFeature> tracks;

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
    SiftPositionReader feature_reader;
    ok = loadList(keypoints_file, keypoints, feature_reader);
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
          tracks[index][u] = IndexedFeature(b, keypoints[b]);
          active_tracks[b] = index;
        } else {
          // It wasn't: create a new track.
          tracks.push_back(Track<IndexedFeature>());
          tracks.back()[t] = IndexedFeature(a, previous_keypoints[a]);
          tracks.back()[u] = IndexedFeature(b, keypoints[b]);
          active_tracks[b] = tracks.size() - 1;
        }
      }

      previous_active_tracks.swap(active_tracks);
    }

    previous_keypoints.swap(keypoints);
    u += 1;
  }

  // Write out matches.
  IndexedFeatureWriter writer;
  ok = saveTrackList(tracks_file, tracks, writer);
  CHECK(ok) << "Could not save tracks to file";

  return 0;
}
