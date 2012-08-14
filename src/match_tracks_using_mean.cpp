#include <cstdlib>
#include <string>
#include <sstream>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "track_list.hpp"
#include "descriptor.hpp"

#include "track_list_reader.hpp"
#include "descriptor_reader.hpp"
#include "reader.hpp"

std::string makeFilename(const std::string& format, int t) {
  return boost::str(boost::format(format) % (t + 1));
}

class NullReader : public Reader<int> {
  public:
    ~NullReader() {}

    void read(const cv::FileNode& node, int& x) {
      x = 0;
    }
};

typedef std::vector<TrackList<int> > TrackListList;

void loadAllTracks(const std::string& format,
                   int num_frames,
                   TrackListList& track_lists) {
  track_lists.clear();

  for (int t = 0; t < num_frames; t += 1) {
    // Read from file.
    TrackList<int> tracks;
    std::string tracks_file = makeFilename(format, t);
    NullReader reader;
    bool ok = loadTrackList(tracks_file, tracks, reader);
    CHECK(ok) << "Could not load tracks";

    // Add to list (without copying).
    track_lists.push_back(TrackList<int>());
    track_lists.back().swap(tracks);
  }
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Matches tracks." << std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " descriptors-format-1 descriptors-format-2 num-frames" <<
      std::endl;
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
  std::string tracks_format1 = argv[1];
  std::string tracks_format2 = argv[2];
  int num_frames = boost::lexical_cast<int>(argv[3]);

  bool ok;

  TrackList<int> tracks1;
  TrackList<int> tracks2;

  // 

  return 0;
}
