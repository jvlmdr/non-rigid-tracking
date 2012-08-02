#include <iostream>
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
#include "track_list.hpp"
#include "rigid_feature.hpp"

typedef TrackList_<RigidFeature> RigidTrackList;
typedef std::vector<RigidTrackList> TrackListList;

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
    RigidTrackList track_list;

    // Attempt to load tracks.
    ReadRigidFeature read;
    ok = track_list.load(tracks_file, read);
    if (!ok) {
      // Failed.
      continue;
    }

    // Put into list.
    track_lists.push_back(RigidTrackList());
    track_lists.back().swap(track_list);

    std::cout << "." << std::flush;
    t += 1;
  }
  std::cout << std::endl;

  return ok;
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Merges rigid-warp tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks-format merged";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string tracks_format = argv[1];
  std::string merged_file = argv[2];

  bool ok;

  std::cout << "loading all tracks" << std::endl;

  // Load tracks from every frame.
  TrackListList track_lists;
  ok = loadAllTracks(tracks_format, track_lists);
  if (!ok) {
    std::cerr << "could not load tracks" << std::endl;
    return 1;
  }
  std::cout << "loaded " << track_lists.size() << " lists of tracks" << std::endl;

  // Now combine all tracks.
  // TODO: Advantageous to enforce that tracks are ordered by their first frame?
  RigidTrackList merged;

  for (TrackListList::const_iterator tracks = track_lists.begin();
       tracks != track_lists.end();
       ++tracks) {
    // Add each track to the full list.
    for (RigidTrackList::const_iterator track = tracks->begin();
         track != tracks->end();
         ++track) {
      merged.push_back(*track);
    }
  }

  WriteRigidFeature write;
  ok = merged.save(merged_file, write);
  if (!ok) {
    std::cerr << "could not save tracks" << std::endl;
    return 1;
  }

  return 0;
}
