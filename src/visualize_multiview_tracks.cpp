#include <string>
#include <cstdlib>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"

#include "multiview_track_list_reader.hpp"
#include "default_reader.hpp"
#include "read_lines.hpp"

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Visualizes multi-view tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks keypoints-format image-format view-names" <<
      std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 5) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string keypoints_format = argv[2];
  std::string image_format = argv[3];
  std::string views_file = argv[4];

  bool ok;

  // Load tracks.
  MultiviewTrackList<int> tracks;
  DefaultReader<int> int_reader;
  ok = loadMultiviewTrackList(tracks_file, tracks, int_reader);
  CHECK(ok) << "Could not load tracks";
  LOG(INFO) << "Loaded " << tracks.numTracks() << " multi-view tracks";

  // Load names of views.
  std::vector<std::string> views;
  ok = readLines(views_file, views);
  CHECK(ok) << "Could not load view names";

  // Ensure that number of views matches.
  int num_views = views.size();
  CHECK(num_views == tracks.numViews());

  return 0;
}
