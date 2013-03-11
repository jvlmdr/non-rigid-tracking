#include <iostream>
#include <fstream>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "tools/using.hpp"
#include "videoseg/segmentation.hpp"
#include "util/random-color.hpp"
#include "track_list.hpp"
#include "track_list_reader.hpp"
#include "image_point_reader.hpp"

const int FOREGROUND_LABEL = 0;
const int BACKGROUND_LABEL = 1;

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Applies a foreground mask to a list of tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks foreground filtered-tracks" << std::endl;
  usage << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  string input_tracks_file = argv[1];
  string foreground_file = argv[2];
  string out_tracks_file = argv[3];

  bool ok;

  // Load tracks from file.
  TrackList<cv::Point2d> input_tracks;
  ImagePointReader<double> feature_reader;
  ok = loadTrackList(input_tracks_file, input_tracks, feature_reader);
  CHECK(ok) << "Could not load tracks";

  // Load segmentation from file.
  videoseg::VideoSegmentation foreground;
  std::ifstream ifs(foreground_file.c_str(), std::ios::binary);
  if (!ifs) {
    LOG(FATAL) << "Could not open foreground segmentation file";
  }
  ok = foreground.ParseFromIstream(&ifs);
  if (!ok) {
    LOG(FATAL) << "Could not parse segmentation from file";
  }

  return 0;
}
