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

Descriptor computeMean(const Track<Descriptor>& descriptors) {
  CHECK(!descriptors.empty()) << "Cannot compute mean of an empty set";

  std::vector<double> total;
  int num_dimensions = 0;

  Track<Descriptor>::const_iterator iter;
  for (iter = descriptors.begin(); iter != descriptors.end(); ++iter) {
    const Descriptor& descriptor = iter->second;

    // Get number of dimensions from first element.
    if (iter == descriptors.begin()) {
      num_dimensions = descriptor.data.size();
      total.assign(num_dimensions, 0.);
    }

    for (int j = 0; j < num_dimensions; j += 1) {
      total[j] += descriptor.data[j];
    }
  }

  int num_descriptors = descriptors.size();
  Descriptor mean(num_dimensions);

  for (int j = 0; j < num_dimensions; j += 1) {
    mean.data[j] = total[j] / num_descriptors;
  }

  return mean;
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

  // Compute mean of each descriptor.
  std::vector<Descriptor> descriptors1;
  std::transform(tracks1.begin(), tracks1.end(),
      std::back_inserter(descriptors1), computeMean);

  std::vector<Descriptor> descriptors2;
  std::transform(tracks2.begin(), tracks2.end(),
      std::back_inserter(descriptors2), computeMean);

  // Match in both directions.
  std::vector<DirectedMatchResult> forward_matches;
  std::vector<DirectedMatchResult> reverse_matches;
  matchBothDirections(descriptors1, descriptors2, forward_matches,
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
