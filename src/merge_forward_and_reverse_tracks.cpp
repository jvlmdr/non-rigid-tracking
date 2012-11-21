#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/lexical_cast.hpp>

#include "track_list.hpp"
#include "scale_space_position.hpp"

#include "track_list_reader.hpp"
#include "track_list_writer.hpp"
#include "scale_space_position_reader.hpp"
#include "scale_space_position_writer.hpp"

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Merges a forward and a reverse track" << std::endl;
  usage << std::endl;
  usage << argv[0] << " forward-tracks reverse-tracks offset tracks" <<
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

  std::string forward_file = argv[1];
  std::string reverse_file = argv[2];
  int offset = boost::lexical_cast<int>(argv[3]);
  std::string tracks_file = argv[4];

  bool ok;

  // Load tracks.
  TrackList<ScaleSpacePosition> forward_tracks;
  TrackList<ScaleSpacePosition> reverse_tracks;
  ScaleSpacePositionReader feature_reader;
  ok = loadTrackList(forward_file, forward_tracks, feature_reader);
  CHECK(ok) << "Could not load forward tracks";
  LOG(INFO) << "Loaded " << forward_tracks.size() << " forward tracks";
  ok = loadTrackList(reverse_file, reverse_tracks, feature_reader);
  CHECK(ok) << "Could not load reverse tracks";
  LOG(INFO) << "Loaded " << reverse_tracks.size() << " reverse tracks";

  int num_tracks = forward_tracks.size();
  CHECK(forward_tracks.size() == reverse_tracks.size());

  // Merge tracks.
  TrackList<ScaleSpacePosition> tracks(num_tracks);

  {
    TrackList<ScaleSpacePosition>::const_iterator forward_track;
    TrackList<ScaleSpacePosition>::iterator track = tracks.begin();

    for (forward_track = forward_tracks.begin();
         forward_track != forward_tracks.end();
         ++forward_track) {
      Track<ScaleSpacePosition>::const_iterator elem;
      for (elem = forward_track->begin();
           elem != forward_track->end();
           ++elem) {
        (*track)[offset + elem->first] = elem->second;
      }

      ++track;
    }
  }

  {
    TrackList<ScaleSpacePosition>::const_iterator reverse_track;
    TrackList<ScaleSpacePosition>::iterator track = tracks.begin();

    for (reverse_track = reverse_tracks.begin();
         reverse_track != reverse_tracks.end();
         ++reverse_track) {
      Track<ScaleSpacePosition>::const_iterator elem;
      for (elem = reverse_track->begin();
           elem != reverse_track->end();
           ++elem) {
        (*track)[offset - elem->first] = elem->second;
      }

      ++track;
    }
  }

  ScaleSpacePositionWriter feature_writer;
  ok = saveTrackList(tracks_file, tracks, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
