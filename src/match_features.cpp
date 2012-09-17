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
#include "find_matches.hpp"
#include "descriptor_reader.hpp"
#include "iterator_reader.hpp"
#include "writer.hpp"
#include "vector_writer.hpp"
#include "match_result_writer.hpp"

DEFINE_double(second_best_ratio, 1.0,
    "Maximum distance relative to second-best match. >= 1 has no effect.");
DEFINE_bool(use_flann, true,
    "Use FLANN (fast but approximate) to find nearest neighbours.");
DEFINE_bool(reciprocal, true,
    "Require matches to be reciprocal between images.");

typedef std::vector<Descriptor> DescriptorList;
typedef std::vector<MatchResult> MatchResultList;
typedef std::vector<DirectedMatchResult> DirectedMatchResultList;

bool isNotDistinctive(const MatchResult& match, double ratio) {
  // Pick nearest second-best match.
  double second_dist = std::min(match.second_dist1, match.second_dist2);

  // The best match must be within a fraction of that.
  double max_dist = ratio * second_dist;

  return !(match.dist <= max_dist);
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
  double SECOND_BEST_RATIO = FLAGS_second_best_ratio;

  bool ok;

  // Load descriptors.
  DescriptorList descriptors1;
  DescriptorList descriptors2;

  DescriptorReader descriptor_reader;
  ok = loadList(descriptors_file1, descriptors1, descriptor_reader);
  CHECK(ok) << "Could not load descriptors";
  LOG(INFO) << "Loaded " << descriptors1.size() << " descriptors";

  ok = loadList(descriptors_file2, descriptors2, descriptor_reader);
  CHECK(ok) << "Could not load descriptors";
  LOG(INFO) << "Loaded " << descriptors2.size() << " descriptors";

  // Match in both directions.
  DirectedMatchResultList forward_matches;
  DirectedMatchResultList reverse_matches;
  matchBothDirections(descriptors1, descriptors2, forward_matches,
      reverse_matches, FLAGS_use_flann);

  // Merge directional matches.
  MatchResultList matches;
  if (FLAGS_reciprocal) {
    // Reduce to a consistent set.
    intersectionOfMatches(forward_matches, reverse_matches, matches);
    LOG(INFO) << "Found " << matches.size() << " reciprocal matches";
  } else {
    // Throw all matches together.
    unionOfMatches(forward_matches, reverse_matches, matches);
    LOG(INFO) << "Found " << matches.size() << " matches";
  }

  // Filter the matches by the distance ratio.
  MatchResultList distinctive;
  std::remove_copy_if(matches.begin(), matches.end(),
      std::back_inserter(distinctive),
      boost::bind(isNotDistinctive, _1, SECOND_BEST_RATIO));
  matches.swap(distinctive);
  LOG(INFO) << "Pruned to " << matches.size() << " distinctive matches";

  MatchResultWriter writer;
  ok = saveList(matches_file, matches, writer);
  CHECK(ok) << "Could not save list of matches";

  return 0;
}
