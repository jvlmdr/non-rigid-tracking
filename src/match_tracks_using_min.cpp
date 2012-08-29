#include <cstdlib>
#include <string>
#include <sstream>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "track_list.hpp"
#include "descriptor.hpp"
#include "match_result.hpp"
#include "find_matches.hpp"

#include "track_list_reader.hpp"
#include "descriptor_reader.hpp"
#include "reader.hpp"
#include "vector_writer.hpp"
#include "match_result_writer.hpp"

std::string makeFilename(const std::string& format, int t) {
  return boost::str(boost::format(format) % (t + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Matches tracks." << std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " descriptors-1 descriptors-2 matches" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

void extractBags(const TrackList<Descriptor>& tracks,
                 std::vector<DescriptorBag>& descriptors) {
  descriptors.clear();

  TrackList<Descriptor>::const_iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Add empty bag.
    descriptors.push_back(DescriptorBag());
    DescriptorBag& bag = descriptors.back();

    // Iterate through track points.
    TrackIterator<Descriptor> point(*track);

    while (!point.end()) {
      // Add descriptor to bag.
      bag.push_back(point.get());
      point.next();
    }
  }
}

int main(int argc, char** argv) {
  init(argc, argv);
  std::string descriptors_file1 = argv[1];
  std::string descriptors_file2 = argv[2];
  std::string matches_file = argv[3];

  bool ok;

  TrackList<Descriptor> tracks1;
  TrackList<Descriptor> tracks2;

  // Load tracks.
  DescriptorReader descriptor_reader;
  ok = loadTrackList(descriptors_file1, tracks1, descriptor_reader);
  CHECK(ok) << "Could not load descriptors";
  LOG(INFO) << "Loaded " << tracks1.size() << " descriptors";
  ok = loadTrackList(descriptors_file2, tracks2, descriptor_reader);
  CHECK(ok) << "Could not load descriptors";
  LOG(INFO) << "Loaded " << tracks2.size() << " descriptors";

  // Convert tracks to sets.
  std::vector<DescriptorBag> descriptors1;
  std::vector<DescriptorBag> descriptors2;
  extractBags(tracks1, descriptors1);
  extractBags(tracks2, descriptors2);

  // Match in both directions.
  std::vector<DirectedMatchResult> forward_matches;
  std::vector<DirectedMatchResult> reverse_matches;
  matchBagsBothDirections(descriptors1, descriptors2, forward_matches,
      reverse_matches, true);

  // Merge directional matches.
  std::vector<MatchResult> matches;
  // Reduce to a consistent set.
  intersectionOfMatches(forward_matches, reverse_matches, matches);
  LOG(INFO) << "Found " << matches.size() << " reciprocal matches";

  MatchResultWriter writer;
  ok = saveList(matches_file, matches, writer);
  CHECK(ok) << "Could not save list of matches";

  return 0;
}
