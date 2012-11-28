#include <string>
#include <sstream>
#include <vector>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "match.hpp"
#include "track_list.hpp"
#include "scale_space_position.hpp"
#include "multiview_track_list.hpp"

#include "iterator_reader.hpp"
#include "match_reader.hpp"
#include "track_list_reader.hpp"
#include "scale_space_position_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "scale_space_position_writer.hpp"

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
  std::vector<Match> matches;
  TrackList<ScaleSpacePosition> track_list1;
  TrackList<ScaleSpacePosition> track_list2;

  // Load matches.
  MatchReader match_reader;
  ok = loadList(matches_file, matches, match_reader);
  CHECK(ok) << "Could not load matches";

  // Load tracks.
  ScaleSpacePositionReader feature_reader;
  ok = loadTrackList(tracks_file1, track_list1, feature_reader);
  CHECK(ok) << "Could not load tracks";
  ok = loadTrackList(tracks_file2, track_list2, feature_reader);
  CHECK(ok) << "Could not load tracks";

  // Move tracks into maps.
  std::map<int, Track<ScaleSpacePosition> > tracks1;
  std::map<int, Track<ScaleSpacePosition> > tracks2;

  int n1 = track_list1.size();
  for (int i = 0; i < n1; i += 1) {
    tracks1[i].swap(track_list1[i]);
  }
  int n2 = track_list2.size();
  for (int i = 0; i < n2; i += 1) {
    tracks2[i].swap(track_list2[i]);
  }

  // Create multi-view track.
  int num_views = 2;
  MultiviewTrackList<ScaleSpacePosition> multiview_tracks(num_views);

  // Add multiview track for each match.
  int num_matches = matches.size();
  for (int i = 0; i < num_matches; i += 1) {
    // Move tracks into multiview structure.
    std::map<int, Track<ScaleSpacePosition> >::iterator track1 =
        tracks1.find(matches[i].first);
    CHECK(track1 != tracks1.end()) << "Track already used";
    std::map<int, Track<ScaleSpacePosition> >::iterator track2 =
        tracks2.find(matches[i].first);
    CHECK(track2 != tracks2.end()) << "Track already used";

    MultiviewTrack<ScaleSpacePosition> track(2);
    track.view(0).swap(track1->second);
    track.view(1).swap(track2->second);

    multiview_tracks.push_back(MultiviewTrack<ScaleSpacePosition>());
    multiview_tracks.back().swap(track);

    // Remove entry in map.
    tracks1.erase(track1);
    tracks2.erase(track2);
  }

  LOG(INFO) << "Multi-view: " << multiview_tracks.numTracks();
  LOG(INFO) << "First view only: " << tracks1.size();
  LOG(INFO) << "Second view only: " << tracks2.size();

  // Add all remaining tracks as their own multiview track.
  while (!tracks1.empty()) {
    std::map<int, Track<ScaleSpacePosition> >::iterator track1 =
        tracks1.begin();

    MultiviewTrack<ScaleSpacePosition> track(2);
    track.view(0).swap(track1->second);

    multiview_tracks.push_back(MultiviewTrack<ScaleSpacePosition>());
    multiview_tracks.back().swap(track);

    tracks1.erase(track1);
  }

  // Add all remaining tracks as their own multiview track.
  while (!tracks2.empty()) {
    std::map<int, Track<ScaleSpacePosition> >::iterator track2 =
        tracks2.begin();

    MultiviewTrack<ScaleSpacePosition> track(2);
    track.view(1).swap(track2->second);

    multiview_tracks.push_back(MultiviewTrack<ScaleSpacePosition>());
    multiview_tracks.back().swap(track);

    tracks2.erase(track2);
  }

  LOG(INFO) << "Total: " << multiview_tracks.numTracks();

  // Save track list.
  ScaleSpacePositionWriter writer;
  ok = saveMultiviewTrackList(multiview_tracks_file, multiview_tracks, writer);
  CHECK(ok) << "Could not save multi-view tracks";

  return 0;
}
