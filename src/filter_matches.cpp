#include <string>
#include <vector>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "match_result.hpp"
#include "unique_match_result.hpp"

#include "iterator_reader.hpp"
#include "match_result_reader.hpp"
#include "unique_match_result_reader.hpp"

#include "iterator_writer.hpp"
#include "match_result_writer.hpp"

DEFINE_bool(unique, true,
    "Each feature is associated to at most one other feature");

DEFINE_bool(use_clearance, false,
    "Require the best match to be sufficiently far from the second-best");
DEFINE_double(clearance, 1.0,
    "The minimum clearance to be considered distinctive. 0.7 or 0.8 is "
    "typical, >= 1 has no effect");

DEFINE_bool(use_absolute_threshold, false,
    "Require that the distance between matches is below a threshold? Note that "
    "1 corresponds to a positive classification, since distance = exp(-score)");
DEFINE_double(absolute_threshold, 1, "The absolute distance threshold");

//DEFINE_bool(use_relative_threshold, false,
//    "Keep matches that are within a fraction of the distance of the best "
//    "match");
//DEFINE_double(relative_threshold, 0.7, "The relative distance threshold");

////////////////////////////////////////////////////////////////////////////////

bool matchIsUndistinctive(const UniqueMatchResult& match, double ratio) {
  // Pick nearest second-best match.
  double next_best = match.minNextBest();
  // The best match must be within a fraction of that.
  double max_distance = ratio * next_best;

  return !(match.distance <= max_distance);
}

void selectDistinctiveMatches(const std::vector<UniqueMatchResult>& matches,
                              std::vector<UniqueMatchResult>& distinctive,
                              double ratio) {
  distinctive.clear();
  std::remove_copy_if(matches.begin(), matches.end(),
      std::back_inserter(distinctive),
      boost::bind(matchIsUndistinctive, _1, ratio));
}

void removeUndistinctiveMatches(std::vector<UniqueMatchResult>& matches,
                                double ratio,
                                bool forward) {
  std::vector<UniqueMatchResult> distinctive;
  selectDistinctiveMatches(matches, distinctive, ratio);

  if (forward) {
    LOG(INFO) << "Kept " << distinctive.size() << " / " << matches.size() <<
        " distinctive forward matches";
  } else {
    LOG(INFO) << "Kept " << distinctive.size() << " / " << matches.size() <<
        " distinctive reverse matches";
  }

  matches.swap(distinctive);
}

////////////////////////////////////////////////////////////////////////////////

bool matchIsPoor(const MatchResult& match, double threshold) {
  return match.distance > threshold;
}

void selectGoodMatches(const std::vector<MatchResult>& matches,
                       std::vector<MatchResult>& good,
                       double threshold) {
  good.clear();
  std::remove_copy_if(matches.begin(), matches.end(), std::back_inserter(good),
      boost::bind(matchIsPoor, _1, threshold));
}

void removePoorMatches(std::vector<MatchResult>& matches,
                       double threshold,
                       bool forward) {
  std::vector<MatchResult> good;
  selectGoodMatches(matches, good, threshold);

  if (forward) {
    LOG(INFO) << "Kept " << good.size() << " / " << matches.size() <<
        " good forward matches";
  } else {
    LOG(INFO) << "Kept " << good.size() << " / " << matches.size() <<
        " good reverse matches";
  }

  matches.swap(good);
}

////////////////////////////////////////////////////////////////////////////////

MatchResult uniqueResultToResult(const UniqueMatchResult& result) {
  return MatchResult(result.index1, result.index2, result.distance);
}

////////////////////////////////////////////////////////////////////////////////

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Computes matches between sets of descriptors." << std::endl;
  usage << std::endl;
  usage << argv[0] << " input-matches output-matches" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string input_file = argv[1];
  std::string output_file = argv[2];

  bool ok;

  std::vector<MatchResult> matches;

  if (FLAGS_unique) {
    // If unique matches were found, can threshold clearance.
    std::vector<UniqueMatchResult> unique_matches;

    UniqueMatchResultReader match_reader;
    ok = loadList(input_file, unique_matches, match_reader);
    CHECK(ok) << "Could not load matches";
    LOG(INFO) << "Loaded " << unique_matches.size() << " matches";

    if (FLAGS_use_clearance) {
      // Filter results that are not distinctive.
      removeUndistinctiveMatches(unique_matches, true, FLAGS_clearance);
    }

    // Convert to plain old matches (without next-best information).
    std::transform(unique_matches.begin(), unique_matches.end(),
        std::back_inserter(matches), uniqueResultToResult);
  } else {
    MatchResultReader match_reader;
    ok = loadList(input_file, matches, match_reader);
    CHECK(ok) << "Could not load forward matches";
    LOG(INFO) << "Loaded " << matches.size() << " forward matches";
  }

  if (FLAGS_use_absolute_threshold) {
    // Filter matches that are below a threshold.
    removePoorMatches(matches, FLAGS_absolute_threshold, true);
  }

  MatchResultWriter match_writer;
  ok = saveList(output_file, matches, match_writer);
  CHECK(ok) << "Could not save list of matches";

  return 0;
}
