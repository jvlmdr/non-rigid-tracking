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
double measureArcLength(const Track_<RigidFeature>& track) {
  double distance = 0;
  cv::Point2d previous;

  Track_<RigidFeature>::const_iterator point;
  for (point = track.begin(); point != track.end(); ++point) {
    // Get current position.
    const RigidFeature& feature = point->second;
    cv::Point2d current(feature.x, feature.y);

    // If this is the first point, we have no previous measurement.
    if (point != track.begin()) {
      distance += cv::norm(current - previous);
    }

    previous = current;
  }

  return distance;
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Selects the most active tracks from a set of tracks." << std::endl;
  usage << std::endl;
  usage << "Sample usage: " << argv[0] << " input-tracks output-tracks"
    " max-num-tracks";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string input_file = argv[1];
  std::string output_file = argv[2];
  int max_num_tracks = boost::lexical_cast<int>(argv[3]);

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
      std::back_inserter(distances), measureArcLength);

  // Sort these scores by inserting them into a map.
  // Scores map to the index of their track.
  typedef std::map<double, int> ScoreIndexMap;
  ScoreIndexMap map;
  // Populate map.
  int i = 0;
  for (DistanceList::const_iterator distance = distances.begin();
       distance != distances.end();
       ++distance) {
    map[*distance] = i;
    i += 1;
  }

  // Read back top scores.
  TrackList_<RigidFeature> output_tracks;
  for (ScoreIndexMap::const_reverse_iterator mapping = map.rbegin();
       mapping != map.rend() && int(output_tracks.size()) < max_num_tracks;
       ++mapping) {
    int index = mapping->second;
    output_tracks.push_back(input_tracks[index]);

    double distance = mapping->first;
    std::cout << distance << std::endl;
  }

  // Write out tracks.
  WriteRigidFeature write;
  ok = output_tracks.save(output_file, write);
  if (!ok) {
    std::cerr << "could not save tracks" << std::endl;
    return 1;
  }

  return 0;
}
