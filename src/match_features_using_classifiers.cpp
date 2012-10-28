#include <vector>
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

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Computes matches between sets of descriptors." << std::endl;
  usage << std::endl;
  usage << argv[0] << " classifiers descriptors matches" << std::endl;
  usage << std::endl;
  usage << "classifiers -- Input" << std::endl;
  usage << "descriptors -- Input" << std::endl;
  usage << "matches -- Output. Pairwise association of indices" << std::endl;
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

  std::string classifiers_file = argv[1];
  std::string descriptors_file = argv[2];
  std::string matches_file = argv[3];

  bool ok;

  // Load classifiers.
  ClassifierReader classifier_reader;
  std::deque<Classifier> classifiers;
  ok = loadList(classifiers_file, classifiers, classifier_reader);
  CHECK(ok) << "Could not load first classifiers file";
  LOG(INFO) << "Loaded " << classifiers.size() << " classifiers";

  // Load descriptors.
  DescriptorReader descriptor_reader;
  std::deque<Descriptor> descriptors;
  ok = loadList(descriptors_file, descriptors, descriptor_reader);
  CHECK(ok) << "Could not load first descriptors file";
  LOG(INFO) << "Loaded " << descriptors.size() << " descriptors";

  // Find best match for every feature in the other image.
  std::vector<UniqueQueryResult> forward_matches;
  findUniqueMatchesUsingClassifiers(classifiers, descriptors, forward_matches);

  std::vector<UniqueMatchResult> matches;
  convertUniqueQueryResultsToMatches(forward_matches, matches, true);

  UniqueMatchResultWriter match_writer;
  ok = saveList(matches_file, matches, match_writer);
  CHECK(ok) << "Could not save list of matches";

  return 0;
}
