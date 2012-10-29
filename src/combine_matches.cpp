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
#include "match_result_reader.hpp"
#include "unique_match_result_reader.hpp"
#include "iterator_reader.hpp"

#include "iterator_writer.hpp"
#include "match_writer.hpp"

DEFINE_bool(unique, true,
    "Each feature is associated to at most one other feature");

DEFINE_bool(reciprocal, false, "Require matches to be reciprocal");

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

MatchResult flipResult(const MatchResult& result) {
  return MatchResult(result.index2, result.index1, result.distance);
}

UniqueMatchResult flipUniqueResult(const UniqueMatchResult& result) {
  return result.flip();
}

Match resultToMatch(const MatchResult& result) {
  return Match(result.index1, result.index2);
}

MatchResult uniqueResultToResult(const UniqueMatchResult& result) {
  return MatchResult(result.index1, result.index2, result.distance);
}

////////////////////////////////////////////////////////////////////////////////

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Computes matches between sets of descriptors." << std::endl;
  usage << std::endl;
  usage << argv[0] << " forward-matches reverse-matches matches" << std::endl;
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

  std::string forward_file = argv[1];
  std::string reverse_file = argv[2];
  std::string matches_file = argv[3];

  bool ok;

  std::vector<MatchResult> forward_results;
  std::vector<MatchResult> reverse_results;

  if (FLAGS_unique) {
    // If unique matches were found, can threshold clearance.
    std::vector<UniqueMatchResult> unique_forward;
    std::vector<UniqueMatchResult> unique_reverse;

    UniqueMatchResultReader match_reader;
    ok = loadList(forward_file, unique_forward, match_reader);
    CHECK(ok) << "Could not load forward matches";
    LOG(INFO) << "Loaded " << unique_forward.size() << " forward matches";
    ok = loadList(reverse_file, unique_reverse, match_reader);
    CHECK(ok) << "Could not load reverse matches";
    LOG(INFO) << "Loaded " << unique_reverse.size() << " reverse matches";

    // Flip order of reverse matches.
    std::transform(unique_reverse.begin(), unique_reverse.end(),
        unique_reverse.begin(), flipUniqueResult);

    if (FLAGS_use_clearance) {
      // Filter results that are not distinctive.
      removeUndistinctiveMatches(unique_forward, true, FLAGS_clearance);
      removeUndistinctiveMatches(unique_reverse, false, FLAGS_clearance);
    }

    // Remove extra information.
    std::transform(unique_forward.begin(), unique_forward.end(),
        std::back_inserter(forward_results), uniqueResultToResult);
    std::transform(unique_reverse.begin(), unique_reverse.end(),
        std::back_inserter(reverse_results), uniqueResultToResult);
  } else {
    MatchResultReader match_reader;
    ok = loadList(matches_file, forward_results, match_reader);
    CHECK(ok) << "Could not load forward matches";
    LOG(INFO) << "Loaded " << forward_results.size() << " forward matches";
    ok = loadList(matches_file, reverse_results, match_reader);
    CHECK(ok) << "Could not load reverse matches";
    LOG(INFO) << "Loaded " << reverse_results.size() << " reverse matches";

    // Flip order of reverse matches.
    std::transform(reverse_results.begin(), reverse_results.end(),
        reverse_results.begin(), flipResult);
  }

  if (FLAGS_use_absolute_threshold) {
    // Filter matches that are below a threshold.
    removePoorMatches(forward_results, FLAGS_absolute_threshold, true);
    removePoorMatches(reverse_results, FLAGS_absolute_threshold, false);
  }

  std::vector<Match> forward_matches;
  std::vector<Match> reverse_matches;

  // Remove extra information.
  std::transform(forward_results.begin(), forward_results.end(),
      std::back_inserter(forward_matches), resultToMatch);
  std::transform(reverse_results.begin(), reverse_results.end(),
      std::back_inserter(reverse_matches), resultToMatch);

  std::vector<Match> matches;
  if (FLAGS_reciprocal) {
    // Reduce to a consistent set.
    intersectionOfMatches(forward_matches, reverse_matches, matches);
    LOG(INFO) << "Found " << matches.size() << " reciprocal matches";
  } else {
    // Throw all matches together.
    unionOfMatches(forward_matches, reverse_matches, matches);
    LOG(INFO) << "Found " << matches.size() << " matches";
  }

  MatchWriter match_writer;
  ok = saveList(matches_file, matches, match_writer);
  CHECK(ok) << "Could not save list of matches";

  return 0;
}
