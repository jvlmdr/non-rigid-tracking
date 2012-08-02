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
#include "track_list.hpp"
#include "rigid_feature.hpp"

// Considers only (x, y) movement not rotation or scale.
double measureAverageStep(const Track_<RigidFeature>& track) {
  double distance = 0;
  int n = 0;
  cv::Point2d previous;

  Track_<RigidFeature>::const_iterator point;
  for (point = track.begin(); point != track.end(); ++point) {
    // Get current position.
    const RigidFeature& feature = point->second;
    cv::Point2d current(feature.x, feature.y);

    // If this is the first point, we have no previous measurement.
    if (point != track.begin()) {
      distance += cv::norm(current - previous);
      n += 1;
    }

    previous = current;
  }

  return distance / n;
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Selects all tracks whose motion is above a threshold." << std::endl;
  usage << std::endl;
  usage << "Sample usage: " << argv[0] << " input-tracks output-tracks"
    " min-distance-per-frame";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string input_file = argv[1];
  std::string output_file = argv[2];
  double min_distance_per_frame = boost::lexical_cast<double>(argv[3]);

  // Load tracks from file.
  TrackList_<RigidFeature> input_tracks;
  ReadRigidFeature read;
  bool ok = input_tracks.load(input_file, read);
  if (!ok) {
    std::cerr << "could not load tracks" << std::endl;
    return 1;
  }

  // Measure how much each track moved.
  typedef std::vector<double> DistanceList;
  DistanceList distances;
  std::transform(input_tracks.begin(), input_tracks.end(),
      std::back_inserter(distances), measureAverageStep);

  // Filter out distances which are too small.
  TrackList_<RigidFeature> output_tracks;
  {
    TrackList_<RigidFeature>::iterator track = input_tracks.begin();
    DistanceList::const_iterator distance = distances.begin();
    while (track != input_tracks.end()) {
      if (*distance >= min_distance_per_frame) {
        output_tracks.push_back(*track);
      }

      ++track;
      ++distance;
    }
  }

  std::cerr << "kept " << output_tracks.size() << " / " <<
    input_tracks.size() << " tracks" << std::endl;

  // Write out tracks.
  WriteRigidFeature write;
  ok = output_tracks.save(output_file, write);
  if (!ok) {
    std::cerr << "could not save tracks" << std::endl;
    return 1;
  }

  return 0;
}
