#include <string>
#include <sstream>
#include <vector>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "match_result.hpp"
#include "track_list.hpp"
#include "similarity_feature.hpp"
#include "multiview_track_list.hpp"

#include "vector_reader.hpp"
#include "match_result_reader.hpp"
#include "track_list_reader.hpp"
#include "similarity_feature_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "similarity_feature_writer.hpp"

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Converts track matches into multi-view tracks." << std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " matches tracks-1 tracks-2 multiview-tracks" << std::endl;
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
  std::string matches_file = argv[1];
  std::string tracks_file1 = argv[2];
  std::string tracks_file2 = argv[3];
  std::string multiview_tracks_file = argv[4];

  bool ok;
  std::vector<MatchResult> matches;
  TrackList<SimilarityFeature> tracks1;
  TrackList<SimilarityFeature> tracks2;

  // Load matches.
  MatchResultReader match_reader;
  ok = loadList(matches_file, matches, match_reader);
  CHECK(ok) << "Could not load matches";

  // Load tracks.
  SimilarityFeatureReader feature_reader;
  ok = loadTrackList(tracks_file1, tracks1, feature_reader);
  CHECK(ok) << "Could not load tracks";
  ok = loadTrackList(tracks_file2, tracks2, feature_reader);
  CHECK(ok) << "Could not load tracks";

  // Create multi-view track.
  int num_views = 2;
  MultiviewTrackList<SimilarityFeature> multiview_tracks(num_views);

  // Add multiview track for each match.
  int num_matches = matches.size();
  for (int i = 0; i < num_matches; i += 1) {
    MultiviewTrack<SimilarityFeature> track(2);
    track.setTrack(0, tracks1[matches[i].index1]);
    track.setTrack(1, tracks2[matches[i].index2]);

    multiview_tracks.add(track);
  }

  // Save track list.
  SimilarityFeatureWriter writer;
  ok = saveMultiviewTrackList(multiview_tracks_file, multiview_tracks, writer);
  CHECK(ok) << "Could not save multi-view tracks";

  return 0;
}
