#include <string>
#include <vector>
#include <cstdlib>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "match_result.hpp"
#include "iterator_reader.hpp"
#include "match_result_reader.hpp"
#include "vector_writer.hpp"
#include "match_result_writer.hpp"

typedef std::vector<MatchResult> MatchList;

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Selects all matches that are sufficiently distinctive." <<
      std::endl;
  usage << std::endl;
  usage << "Sample usage: " << argv[0] << " input-matches output-matches"
      " second-best-ratio" << std::endl;
  usage << std::endl;
  usage << "Ratio is between 0 and 1. 1 allows all, 0 excludes all.";
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

  std::string input_file = argv[1];
  std::string output_file = argv[2];
  double max_relative_distance = boost::lexical_cast<double>(argv[3]);

  // Load matches from file.
  MatchList matches;
  MatchResultReader match_reader;
  bool ok = loadList(input_file, matches, match_reader);
  CHECK(ok) << "Could not load matches";

  // Filter by distinctiveness.
  MatchList distinctive;

  MatchList::const_iterator match;
  for (match = matches.begin(); match != matches.end(); ++match) {
    // Calculate distinctiveness.
    double nearest = std::min(match->second_dist1, match->second_dist2);
    double relative_distance = match->dist / nearest;

    // Only keep if below threshold.
    if (relative_distance < max_relative_distance) {
      distinctive.push_back(*match);
    }
  }

  int num_input = matches.size();
  int num_output = distinctive.size();
  double fraction = static_cast<double>(num_output) / num_input;
  LOG(INFO) << "Kept " << num_output << " / " << num_input << " matches (" <<
      fraction << ")";

  // Write out matches.
  MatchResultWriter match_writer;
  ok = saveList(output_file, distinctive, match_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
