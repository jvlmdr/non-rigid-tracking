#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "read_image.hpp"
#include "descriptor.hpp"
#include "match_result.hpp"
#include "unique_match_result.hpp"
#include "find_matches.hpp"
#include "find_unique_matches.hpp"

#include "descriptor_reader.hpp"
#include "iterator_reader.hpp"
#include "vector_writer.hpp"
#include "match_result_writer.hpp"
#include "unique_match_result_writer.hpp"

DEFINE_bool(unique, true,
    "Each feature is associated to at most one other feature");
// Settings for when unique == true.
DEFINE_double(min_relative_clearance, 1.0,
    "Unique matching. "
    "Specifies the minimum relative distance that the second-best match must "
    "be from the first in order to be considered distinctive. "
    "0.7 or 0.8 is typical, >= 1 has no effect.");
// Settings for when unique == false.
DEFINE_int32(max_num_matches, 0,
    "Non-unique matching. Maximum number of matches. "
    "Unused if non-positive.");
DEFINE_double(max_relative_distance, 0.,
    "Non-unique matching. "
    "Return all matches within this fraction of the nearest match. "
    "0.9 is typical, 0 means all matches, 1 means only the nearest.");

DEFINE_bool(reciprocal, true, "Require matches to be reciprocal.");
DEFINE_bool(use_flann, true,
    "Use FLANN (fast but approximate) to find nearest neighbours.");

bool isNotDistinctive(const UniqueMatchResult& match, double ratio) {
  // Pick nearest second-best match.
  double next_best = match.minNextBest();
  // The best match must be within a fraction of that.
  double max_distance = ratio * next_best;

  return !(match.distance <= max_distance);
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Computes matches between sets of descriptors." << std::endl;
  usage << std::endl;
  usage << argv[0] << " descriptors1 descriptors2 matches" << std::endl;
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
  double SECOND_BEST_RATIO = FLAGS_min_relative_clearance;

  bool ok;

  // Load descriptors.
  DescriptorReader descriptor_reader;
  std::vector<Descriptor> descriptors1;
  ok = loadList(descriptors_file1, descriptors1, descriptor_reader);
  CHECK(ok) << "Could not load first descriptors file";
  LOG(INFO) << "Loaded " << descriptors1.size() << " descriptors";

  std::vector<Descriptor> descriptors2;
  ok = loadList(descriptors_file2, descriptors2, descriptor_reader);
  CHECK(ok) << "Could not load second descriptors file";
  LOG(INFO) << "Loaded " << descriptors2.size() << " descriptors";

  if (FLAGS_unique) {
    // Find one match in each direction.
    std::vector<UniqueDirectedMatch> forward_matches;
    std::vector<UniqueDirectedMatch> reverse_matches;
    matchUniqueBothDirections(descriptors1, descriptors2, forward_matches,
        reverse_matches, FLAGS_use_flann);

    // Merge directional matches.
    std::vector<UniqueMatchResult> matches;
    if (FLAGS_reciprocal) {
      // Reduce to a consistent set.
      intersectionOfUniqueMatches(forward_matches, reverse_matches, matches);
      LOG(INFO) << "Found " << matches.size() << " reciprocal matches";
    } else {
      // Throw all matches together.
      unionOfUniqueMatches(forward_matches, reverse_matches, matches);
      LOG(INFO) << "Found " << matches.size() << " matches";
    }

    // Filter the matches by the distance ratio.
    std::vector<UniqueMatchResult> distinctive;
    std::remove_copy_if(matches.begin(), matches.end(),
        std::back_inserter(distinctive),
        boost::bind(isNotDistinctive, _1, SECOND_BEST_RATIO));
    matches.swap(distinctive);
    LOG(INFO) << "Pruned to " << matches.size() << " distinctive matches";

    UniqueMatchResultWriter writer;
    ok = saveList(matches_file, matches, writer);
    CHECK(ok) << "Could not save list of matches";
  } else {
    // Find several matches in each direction.
    std::vector<DirectedMatchList> forward_matches;
    std::vector<DirectedMatchList> reverse_matches;
    matchBothDirections(descriptors1, descriptors2, forward_matches,
        reverse_matches, FLAGS_max_num_matches, FLAGS_max_relative_distance,
        FLAGS_use_flann);

    // Merge directional matches.
    std::vector<MatchResult> matches;
    if (FLAGS_reciprocal) {
      // Reduce to a consistent set.
      intersectionOfMatchLists(forward_matches, reverse_matches, matches);
      LOG(INFO) << "Found " << matches.size() << " reciprocal matches";
    } else {
      // Throw all matches together.
      unionOfMatchLists(forward_matches, reverse_matches, matches);
      LOG(INFO) << "Found " << matches.size() << " matches";
    }

    MatchResultWriter writer;
    ok = saveList(matches_file, matches, writer);
    CHECK(ok) << "Could not save list of matches";
  }

  return 0;
}
