#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "multiview_track_list.hpp"
#include "similarity_feature.hpp"
#include "similarity_feature_reader.hpp"
#include "multiview_track_list_reader.hpp"
#include "similarity_feature_writer.hpp"
#include "multiview_track_list_writer.hpp"

DEFINE_bool(top_n, false, "Select best n tracks (versus a threshold)");
DEFINE_bool(fraction, false, "When top_n is enabled, selects top fraction");

void pathLength(const Track<SimilarityFeature>& track,
                double& distance,
                int& duration) {
  distance = 0;
  duration = 0;

  int previous_time = 0;
  cv::Point2d previous_position;
  TrackIterator<SimilarityFeature> iterator(track);

  while (!iterator.end()) {
    int time = iterator.time();
    cv::Point2d position(iterator.get().x, iterator.get().y);

    // Skip first frame.
    if (!iterator.begin()) {
      // Measure distance
      distance += cv::norm(position - previous_position);
      duration += (time - previous_time);
    }

    previous_time = time;
    previous_position = position;
    iterator.next();
  }
}

double measureAverageStep(const MultiviewTrack<SimilarityFeature>& track) {
  double total_distance = 0;
  int total_duration = 0;

  for (int view = 0; view < track.numViews(); view += 1) {
    double distance;
    int duration;

    pathLength(track.view(view), distance, duration);

    total_distance += distance;
    total_duration += duration;
  }

  return total_distance / total_duration;
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Selects all tracks whose motion is above a threshold." << std::endl;
  usage << std::endl;
  usage << "Sample usage: " << argv[0] << " input-tracks output-tracks"
    " min-distance-per-frame";
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

bool greaterThan(double x, double y) {
  return x > y;
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string input_file = argv[1];
  std::string output_file = argv[2];
  const char* threshold = argv[3];

  // Load tracks from file.
  MultiviewTrackList<SimilarityFeature> input_tracks;
  SimilarityFeatureReader feature_reader;
  bool ok = loadMultiviewTrackList(input_file, input_tracks, feature_reader);
  CHECK(ok) << "Could not load tracks";

  int num_tracks = input_tracks.numTracks();
  double min_distance_per_frame;

  // Measure how much each point moved.
  std::vector<double> distances;
  std::transform(input_tracks.begin(), input_tracks.end(),
      std::back_inserter(distances), measureAverageStep);

  // Take tracks as long as the n-th value or x-th percentile.
  // (To avoid non-deterministic behaviour.)
  if (FLAGS_top_n) {
    // Sort tracks by how much they move.
    std::vector<double> sorted_distances(distances);
    std::sort(sorted_distances.begin(), sorted_distances.end(), greaterThan);

    int n;
    if (FLAGS_fraction) {
      double x = boost::lexical_cast<double>(threshold);
      CHECK(x >= 0);
      CHECK(x <= 1);
      n = static_cast<int>(x * (num_tracks - 1));
    } else {
      n = boost::lexical_cast<int>(threshold);
      CHECK(n > 0);
    }

    n = std::min(n, num_tracks - 1);
    min_distance_per_frame = sorted_distances[n];
  } else {
    // Simply read threshold from flag.
    min_distance_per_frame = boost::lexical_cast<double>(threshold);
  }

  int num_views = input_tracks.numViews();
  MultiviewTrackList<SimilarityFeature> output_tracks(num_views);

  // Filter out distances which are too small.
  for (int i = 0; i < num_tracks; i += 1) {
    if (distances[i] >= min_distance_per_frame) {
      MultiviewTrack<SimilarityFeature> copy(input_tracks.track(i));
      output_tracks.add(copy);
    }
  }

  int num_input = input_tracks.numTracks();
  int num_output = output_tracks.numTracks();
  double fraction = static_cast<double>(num_output) / num_input;
  LOG(INFO) << "Kept " << num_output << " / " << num_input << " tracks (" <<
      fraction << ")";

  // Write out tracks.
  SimilarityFeatureWriter feature_writer;
  ok = saveMultiviewTrackList(output_file, output_tracks, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
