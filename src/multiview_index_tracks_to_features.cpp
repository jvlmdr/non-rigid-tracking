#include <string>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "multiview_track_list.hpp"
#include "sift_position.hpp"

#include "multiview_track_list_reader.hpp"
#include "vector_reader.hpp"
#include "default_reader.hpp"

DEFINE_bool(input_multitracks, false,
    "Input tracks or multi-tracks of indices?");
DEFINE_bool(input_keypoints, true, "Build output from keypoints or tracks?");

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Builds feature tracks from index tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " index-tracks features feature-tracks" << std::endl;
  usage << std::endl;
  usage << "Capable of processing tracks (one feature per frame) or"
      " multi-tracks (set of features per frame). If input_multitracks is false"
      " and input_keypoints is true then the output will be tracks. Otherwise"
      " it will be multi-tracks." << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

typedef std::vector<int> IndexSet;
typedef std::vector<SiftPosition> FeatureSet;

/*
void tracksToMultitracks(const MultiviewTrackList<int>& tracks,
                         MultiviewTrackList<IndexSet>& multitracks) {
  int num_tracks = tracks.numTracks();
  int num_views = tracks.numViews();

  multitracks.reset(num_views);

  for (int i = 0; i < num_tracks; i += 1) {
    // Initialize 
    MultiviewTrack<IndexSet> multitrack(num_views);

    for (int j = 0; j < num_views; j += 1) {

    // Iterate through points in track.
    typename 

    const MultiviewTrack<int>& track = tracks.track(i);

    // 
  }
}
    // Convert from tracks to multi-tracks.
    std::vector<MultiviewTrack<IndexSet> > multitrack_list(tracks.numTracks());
    for (int i = 0; i < int(tracks.numTracks()); i += 1) {
    }
*/

int main(int argc, char** argv) {
  init(argc, argv);
  std::string index_tracks_file = argv[0];
  std::string features_file = argv[1];
  std::string feature_tracks_file = argv[2];

  bool ok;
  MultiviewTrackList<IndexSet> index_multitracks;

  if (FLAGS_input_multitracks) {
    // Load multi-tracks from file.
    DefaultReader<int> index_reader;
    VectorReader<int> reader(index_reader);
    ok = loadMultiviewTrackList(index_tracks_file, index_multitracks, reader);
    CHECK(ok) << "Could not load multi-tracks";
  } else {
    // Load tracks from file.
    MultiviewTrackList<int> tracks;
    DefaultReader<int> reader;
    ok = loadMultiviewTrackList(index_tracks_file, tracks, reader);
    CHECK(ok) << "Could not load tracks";

  }

  return 0;
}
