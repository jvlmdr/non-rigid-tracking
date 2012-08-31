#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cstdlib>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <gflags/gflags.h>

#include "multiview_track_list.hpp"
#include "sift_position.hpp"

#include "sift_position_reader.hpp"
#include "sift_position_writer.hpp"
#include "multiview_track_list_reader.hpp"
#include "multiview_track_list_writer.hpp"

// Hmm.. this may be shadowing TrackList<T>.
typedef MultiviewTrackList<SiftPosition> TrackList;
// Use a list to allow it to grow dynamically without copying.
typedef std::list<TrackList> TrackListList;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

bool fileExists(const std::string& filename) {
  return std::ifstream(filename.c_str()).good();
}

bool loadAllTracks(const std::string& tracks_format,
                   TrackListList& track_lists) {
  bool ok = true;
  bool have_frame = true;
  int t = 0;

  while (have_frame && ok) {
    // Check if file exists.
    std::string tracks_file = makeFilename(tracks_format, t);
    have_frame = fileExists(tracks_file);
    if (!have_frame) {
      // End of movie.
      continue;
    }

    // Add an empty track list.
    TrackList track_list;

    // Attempt to load tracks.
    SiftPositionReader feature_reader;
    ok = loadMultiviewTrackList(tracks_file, track_list, feature_reader);
    if (!ok) {
      // Failed.
      continue;
    }

    LOG(INFO) << "Read " << track_list.numTracks() << " tracks from `" <<
        tracks_file << "'";

    // Put into list.
    track_lists.push_back(TrackList());
    track_lists.back().swap(track_list);

    t += 1;
  }

  return ok;
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Merges multiview tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks-format merged";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_format = argv[1];
  std::string merged_file = argv[2];

  bool ok;

  // Load tracks from every frame.
  TrackListList track_lists;
  ok = loadAllTracks(tracks_format, track_lists);
  CHECK(ok) << "Could not load all tracks";
  LOG(INFO) << "Loaded " << track_lists.size() << " lists of tracks";

  // Now combine all tracks.
  // TODO: Get number of views from first non-empty list?
  int num_views = track_lists.front().numViews();
  TrackList merged(num_views);

  for (TrackListList::const_iterator tracks = track_lists.begin();
       tracks != track_lists.end();
       ++tracks) {
    // Add each track to the full list.
    TrackList::const_iterator track;
    for (track = tracks->begin(); track != tracks->end(); ++track) {
      MultiviewTrack<SiftPosition> copy(*track);
      merged.add(copy);
    }
  }

  SiftPositionWriter feature_writer;
  ok = saveMultiviewTrackList(merged_file, merged, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
