#include <cstdlib>
#include <sstream>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "descriptor.hpp"
#include "classifier.hpp"
#include "find_unique_matches.hpp"

#include "iterator_reader.hpp"
#include "descriptor_reader.hpp"
#include "classifier_reader.hpp"

#include "iterator_writer.hpp"
#include "unique_match_result_writer.hpp"

DEFINE_bool(reciprocal, false,
    "Only accept a match if the features are each other's best match?");
DEFINE_bool(directed_consistent, false,
    "Find matches which are forward-consistent (features in the second image may\n"
    "belong to the same track but features in the first may not).");

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Computes matches between sets of descriptors." << std::endl;
  usage << std::endl;
  usage << argv[0] << " descriptors1 descriptors2 classifiers1 classifiers2 "
      "matches" << std::endl;
  usage << std::endl;
  usage << argv[0] << " descriptors1 descriptors2 classifiers1 classifiers2 "
      "matches1 matches2" << std::endl;
  usage << std::endl;
  usage << "descriptors1, descriptors2 -- Input. Descriptors to match." <<
    std::endl;
  usage << "classifiers1, classifiers2 -- Input. Classifiers to use for "
      "metric." << std::endl;
  usage << "matches -- Output. Pairwise association of indices." << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  bool ok = (!FLAGS_directed_consistent && argc == 6) ||
    (FLAGS_directed_consistent && argc == 7);

  if (!ok) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string descriptors_file1 = argv[1];
  std::string descriptors_file2 = argv[2];
  std::string classifiers_file1 = argv[3];
  std::string classifiers_file2 = argv[4];

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

  // Load classifiers.
  ClassifierReader classifier_reader;
  std::deque<Classifier> classifiers1;
  ok = loadList(classifiers_file1, classifiers1, classifier_reader);
  CHECK(ok) << "Could not load first classifiers file";
  LOG(INFO) << "Loaded " << classifiers1.size() << " classifiers";

  std::deque<Classifier> classifiers2;
  ok = loadList(classifiers_file2, classifiers2, classifier_reader);
  CHECK(ok) << "Could not load second classifiers file";
  LOG(INFO) << "Loaded " << classifiers2.size() << " classifiers";

  // Find best match for each feature in the first image in the second image.
  std::vector<UniqueDirectedMatch> forward_matches;
  findUniqueMatchesUsingClassifiers(classifiers1, descriptors2,
      forward_matches);

  // Find best match for each feature in the second image in the first image.
  std::vector<UniqueDirectedMatch> reverse_matches;
  findUniqueMatchesUsingClassifiers(classifiers2, descriptors1,
      reverse_matches);

  // Combine matches in either direction.
  if (FLAGS_directed_consistent) {
    std::string matches_file1 = argv[5];
    std::string matches_file2 = argv[6];

    std::vector<UniqueMatchResult> forward_consistent;
    forwardConsistentUniqueMatches(forward_matches, reverse_matches,
        forward_consistent);
    LOG(INFO) << "Found " << forward_consistent.size() <<
        " forward-consistent matches";

    std::vector<UniqueMatchResult> reverse_consistent;
    forwardConsistentUniqueMatches(reverse_matches, forward_matches,
        reverse_consistent);
    LOG(INFO) << "Found " << reverse_consistent.size() <<
        " reverse-consistent matches";

    UniqueMatchResultWriter match_writer;
    ok = saveList(matches_file1, forward_consistent, match_writer);
    CHECK(ok) << "Could not save list of matches";
    ok = saveList(matches_file2, reverse_consistent, match_writer);
    CHECK(ok) << "Could not save list of matches";
  } else {
    std::string matches_file = argv[5];

    std::vector<UniqueMatchResult> matches;
    if (FLAGS_reciprocal) {
      intersectionOfUniqueMatches(forward_matches, reverse_matches, matches);
    } else {
      unionOfUniqueMatches(forward_matches, reverse_matches, matches);
    }
    LOG(INFO) << "Found " << matches.size() << " matches";

    UniqueMatchResultWriter match_writer;
    ok = saveList(matches_file, matches, match_writer);
    CHECK(ok) << "Could not save list of matches";
  }

  return 0;
}
