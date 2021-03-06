#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "descriptor.hpp"
#include "match_result.hpp"
#include "unique_match_result.hpp"
#include "find_matches.hpp"
#include "find_unique_matches.hpp"

#include "descriptor_reader.hpp"
#include "iterator_reader.hpp"

#include "iterator_writer.hpp"
#include "match_result_writer.hpp"
#include "unique_match_result_writer.hpp"

DEFINE_bool(unique, false, "Only take best match");

DEFINE_bool(use_max_num, false, "Limit number of matches");
DEFINE_int32(max_num, 1, "Maximum number of matches");

DEFINE_bool(use_absolute_threshold, false, "Use absolute distance threshold");
DEFINE_double(absolute_threshold, 1,
    "Maximum appearance distance between features");

DEFINE_bool(use_flann, true,
    "Use FLANN (fast but approximate) to find nearest neighbours.");

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Computes matches between sets of descriptors." << std::endl;
  usage << std::endl;
  usage << argv[0] << " descriptors1 descriptors2 matches1 matches2" <<
      std::endl;
  usage << std::endl;
  usage << "descriptors1, descriptors2 -- Input. Descriptors to match." <<
      std::endl;
  usage << "matches -- Output. Pairwise association of indices." << std::endl;
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

  std::string descriptors_file1 = argv[1];
  std::string descriptors_file2 = argv[2];
  std::string matches_file = argv[3];

  bool ok;

  // Load descriptors.
  DescriptorReader descriptor_reader;
  std::deque<Descriptor> descriptors1;
  ok = loadList(descriptors_file1, descriptors1, descriptor_reader);
  CHECK(ok) << "Could not load first descriptors file";
  LOG(INFO) << "Loaded " << descriptors1.size() << " descriptors";

  std::deque<Descriptor> descriptors2;
  ok = loadList(descriptors_file2, descriptors2, descriptor_reader);
  CHECK(ok) << "Could not load second descriptors file";
  LOG(INFO) << "Loaded " << descriptors2.size() << " descriptors";

  if (FLAGS_unique) {
    // Find one match in each direction.
    std::vector<UniqueQueryResult> forward_matches;
    findUniqueMatchesUsingEuclideanDistance(descriptors1, descriptors2,
        forward_matches, FLAGS_use_flann);

    // Convert from query to match representation.
    std::vector<UniqueMatchResult> matches;
    convertUniqueQueryResultsToMatches(forward_matches, matches, true);
    LOG(INFO) << "Found " << matches.size() << " matches";

    UniqueMatchResultWriter writer;
    ok = saveList(matches_file, matches, writer);
    CHECK(ok) << "Could not save list of matches";
  } else {
    // Find several matches in each direction.
    std::deque<QueryResultList> forward_matches;
    findMatchesUsingEuclideanDistance(descriptors1, descriptors2,
        forward_matches, FLAGS_use_max_num, FLAGS_max_num,
        FLAGS_use_absolute_threshold, FLAGS_absolute_threshold,
        FLAGS_use_flann);

    // Flatten out lists of query results to match results.
    std::vector<MatchResult> matches;
    convertQueryResultListsToMatches(forward_matches, matches, true);
    LOG(INFO) << "Found " << matches.size() << " matches";

    MatchResultWriter writer;
    ok = saveList(matches_file, matches, writer);
    CHECK(ok) << "Could not save list of matches";
  }

  return 0;
}
