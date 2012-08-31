#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "track_list.hpp"
#include "sift_position.hpp"

#include "sift_position_reader.hpp"
#include "sift_position_writer.hpp"
#include "track_list_reader.hpp"
#include "track_list_writer.hpp"

DEFINE_bool(top_n, false, "Select best n tracks (versus a threshold)");
DEFINE_bool(fraction, false, "When top_n is enabled, selects top fraction");

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Selects all tracks longer than a given threshold." << std::endl;
  usage << std::endl;
  usage << "Sample usage: " << argv[0] << " input-tracks output-tracks"
    " threshold";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
}

struct ScoredIndex {
  int index;
  int score;

  ScoredIndex(int index, int score) : index(index), score(score) {}

  bool operator<(const ScoredIndex& other) const {
    return score > other.score;
  }
};

int main(int argc, char** argv) {
  init(argc, argv);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }
  std::string input_file = argv[1];
  std::string output_file = argv[2];

  // Load tracks from file.
  TrackList<SiftPosition> input_tracks;
  SiftPositionReader feature_reader;
  bool ok = loadTrackList(input_file, input_tracks, feature_reader);
  CHECK(ok) << "Could not load tracks";

  int num_tracks = input_tracks.size();
  int min_track_length;

  // Take tracks as long as the n-th value or x-th percentile.
  // (To avoid non-deterministic behaviour.)
  if (FLAGS_top_n) {
    // Sort tracks by their length.
    std::vector<ScoredIndex> scored;
    for (int i = 0; i < int(input_tracks.size()); i += 1) {
      const Track<SiftPosition>& track = input_tracks[i];
      int length = track.rbegin()->first - track.begin()->first + 1;
      scored.push_back(ScoredIndex(i, length));
    }
    std::sort(scored.begin(), scored.end());

    int n;
    if (FLAGS_fraction) {
      double x = boost::lexical_cast<double>(argv[3]);
      CHECK(x >= 0);
      CHECK(x <= 1);
      n = static_cast<int>(x * (num_tracks - 1));
    } else {
      n = boost::lexical_cast<int>(argv[3]);
      CHECK(n > 0);
    }

    n = std::min(n, num_tracks - 1);
    min_track_length = scored[n].score;
  } else {
    // Simply read threshold from flag.
    min_track_length = boost::lexical_cast<int>(argv[3]);
  }

  TrackList<SiftPosition> output_tracks;

  TrackList<SiftPosition>::const_iterator track;
  for (track = input_tracks.begin(); track != input_tracks.end(); ++track) {
    // Measure track length.
    int length = track->rbegin()->first - track->begin()->first + 1;

    // Only keep if above threshold.
    if (length >= min_track_length) {
      output_tracks.push_back(*track);
    }
  }

  int num_input = input_tracks.size();
  int num_output = output_tracks.size();
  double fraction = static_cast<double>(num_output) / num_input;
  LOG(INFO) << "Kept " << num_output << " / " << num_input << " tracks (" <<
      fraction << ")";

  // Write out tracks.
  SiftPositionWriter feature_writer;
  ok = saveTrackList(output_file, output_tracks, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
