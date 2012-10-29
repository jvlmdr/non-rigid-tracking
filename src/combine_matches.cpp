#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "match.hpp"
#include "match_result.hpp"

#include "iterator_reader.hpp"
#include "match_reader.hpp"
#include "match_result_reader.hpp"

#include "iterator_writer.hpp"
#include "match_writer.hpp"
#include "match_result_writer.hpp"

DEFINE_bool(reciprocal, false, "Require matches to be reciprocal?");
DEFINE_bool(directed_consistent, false,
    "Find matches which are consistent in the first image only");
DEFINE_bool(keep_distance, false,
    "Keep distance labels? (Fails if distance is not symmetric)");

void intersectionOfMatches(const std::vector<Match>& forward,
                           const std::vector<Match>& reverse,
                           std::vector<Match>& matches) {
  std::vector<Match> forward_set = forward;
  std::vector<Match> reverse_set = reverse;

  // Sort and merge.
  std::sort(forward_set.begin(), forward_set.end());
  std::sort(reverse_set.begin(), reverse_set.end());

  matches.clear();
  std::set_intersection(forward_set.begin(), forward_set.end(),
      reverse_set.begin(), reverse_set.end(), std::back_inserter(matches));
}

void unionOfMatches(const std::vector<Match>& forward,
                    const std::vector<Match>& reverse,
                    std::vector<Match>& matches) {
  std::vector<Match> forward_set = forward;
  std::vector<Match> reverse_set = reverse;

  // Sort and merge.
  std::sort(forward_set.begin(), forward_set.end());
  std::sort(reverse_set.begin(), reverse_set.end());
  matches.clear();
  std::set_union(forward_set.begin(), forward_set.end(), reverse_set.begin(),
      reverse_set.end(), std::back_inserter(matches));
}

std::pair<int, int> matchToPair(const Match& match) {
  return std::make_pair(match.first, match.second);
}

std::pair<int, int> matchToReversePair(const Match& match) {
  return std::make_pair(match.second, match.first);
}

Match pairToMatch(const std::pair<int, int>& pair) {
  return Match(pair.first, pair.second);
}

template<class Key, class Value>
bool compareKey(const std::pair<Key, Value>& x,
                const std::pair<Key, Value>& y) {
  return x.first < y.first;
}

void forwardConsistentMatches(const std::vector<Match>& forward,
                              const std::vector<Match>& reverse,
                              std::vector<Match>& matches) {
  // Add all matches which do not share a feature in the second image.
  std::map<int, int> matches_set;

  // Build inverted index of forward matches.
  std::multimap<int, int> inverted;
  std::transform(forward.begin(), forward.end(),
      std::inserter(inverted, inverted.begin()), matchToReversePair);

  std::multimap<int, int>::const_iterator pair = inverted.begin();
  while (pair != inverted.end()) {
    // Seek to end of elements with same key.
    int n = 0;
    std::multimap<int, int>::const_iterator next = pair;
    while (next != inverted.end() && next->first == pair->first) {
      n += 1;
      ++next;
    }
    // Exactly one match?
    if (n == 1) {
      matches_set[pair->second] = pair->first;
    }
    // Advance to next feature.
    pair = next;
  }

  // Remove reverse matches for features which are already matched.
  std::map<int, int> reverse_set;
  std::transform(reverse.begin(), reverse.end(),
      std::inserter(reverse_set, reverse_set.begin()), matchToPair);
  std::map<int, int> difference;
  std::set_difference(reverse_set.begin(), reverse_set.end(),
      matches_set.begin(), matches_set.end(),
      std::inserter(difference, difference.begin()), compareKey<int, int>);

  // Add remaining features.
  std::copy(difference.begin(), difference.end(),
      std::inserter(matches_set, matches_set.begin()));

  // Convert to matches.
  std::transform(matches_set.begin(), matches_set.end(),
      std::inserter(matches, matches.begin()), pairToMatch);
}

////////////////////////////////////////////////////////////////////////////////

bool compareMatchIndices(const MatchResult& x, const MatchResult& y) {
  return Match(x.index1, x.index2) < Match(y.index1, y.index2);
}

template<class InputIterator1, class InputIterator2, class OutputIterator>
OutputIterator mapIntersection(InputIterator1 first1,
                               InputIterator1 last1,
                               InputIterator2 first2,
                               InputIterator2 last2,
                               OutputIterator result) {
  while (first1 != last1 && first2 != last2) {
    if (first1->first < first2->first) {
      ++first1;
    } else if (first2->first < first1->first) {
      ++first2;
    } else {
      CHECK(first1->second == first2->second) << "Values did not match";
      *result++ = *first1;
      ++first1;
      ++first2;
    }
  }

  return result;
}

template<class InputIterator1, class InputIterator2, class OutputIterator>
OutputIterator mapUnion(InputIterator1 first1,
                        InputIterator1 last1,
                        InputIterator2 first2,
                        InputIterator2 last2,
                        OutputIterator result) {
  while (true) {
    if (first1 == last1) {
      return std::copy(first2, last2, result);
    }
    if (first2 == last2) {
      return std::copy(first1, last1, result);
    }

    if (first1->first < first2->first) {
      *result++ = *first1++;
    } else if (first2->first < first1->first) {
      *result++ = *first2++;
    } else {
      CHECK(first1->second == first2->second) << "Values did not match";
      *result++ = *first1;
      ++first1;
      ++first2;
    }
  }
}

std::pair<Match, double> matchResultToPair(const MatchResult& match) {
  return std::make_pair(Match(match.index1, match.index2), match.distance);
}

MatchResult pairToMatchResult(const std::pair<Match, double>& pair) {
  return MatchResult(pair.first, pair.second);
}

void intersectionOfMatchResults(const std::vector<MatchResult>& forward,
                                const std::vector<MatchResult>& reverse,
                                std::vector<MatchResult>& matches) {
  // Convert to maps.
  std::map<Match, double> forward_set;
  std::map<Match, double> reverse_set;
  std::transform(forward.begin(), forward.end(),
      std::inserter(forward_set, forward_set.begin()), matchResultToPair);
  std::transform(reverse.begin(), reverse.end(),
      std::inserter(reverse_set, reverse_set.begin()), matchResultToPair);

  // Take intersection of maps.
  std::map<Match, double> match_set;
  mapIntersection(forward_set.begin(), forward_set.end(), reverse_set.begin(),
      reverse_set.end(), std::inserter(match_set, match_set.begin()));

  // Convert to list.
  matches.clear();
  std::transform(match_set.begin(), match_set.end(),
      std::back_inserter(matches), pairToMatchResult);
}

void unionOfMatchResults(const std::vector<MatchResult>& forward,
                         const std::vector<MatchResult>& reverse,
                         std::vector<MatchResult>& matches) {
  // Convert to maps.
  std::map<Match, double> forward_set;
  std::map<Match, double> reverse_set;
  std::transform(forward.begin(), forward.end(),
      std::inserter(forward_set, forward_set.begin()), matchResultToPair);
  std::transform(reverse.begin(), reverse.end(),
      std::inserter(reverse_set, reverse_set.begin()), matchResultToPair);

  // Take union of maps.
  std::map<Match, double> match_set;
  mapUnion(forward_set.begin(), forward_set.end(), reverse_set.begin(),
      reverse_set.end(), std::inserter(match_set, match_set.begin()));

  // Convert to list.
  matches.clear();
  std::transform(match_set.begin(), match_set.end(),
      std::back_inserter(matches), pairToMatchResult);
}

////////////////////////////////////////////////////////////////////////////////

Match flipMatch(const Match& match) {
  return Match(match.second, match.first);
}

MatchResult flipResult(const MatchResult& result) {
  return MatchResult(result.index2, result.index1, result.distance);
}

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

  if (FLAGS_keep_distance) {
    std::vector<MatchResult> forward;
    std::vector<MatchResult> reverse;

    MatchResultReader match_reader;
    ok = loadList(forward_file, forward, match_reader);
    CHECK(ok) << "Could not load forward matches";
    LOG(INFO) << "Loaded " << forward.size() << " forward matches";
    ok = loadList(reverse_file, reverse, match_reader);
    CHECK(ok) << "Could not load reverse matches";
    LOG(INFO) << "Loaded " << reverse.size() << " reverse matches";

    // Flip order of reverse matches.
    std::transform(reverse.begin(), reverse.end(), reverse.begin(), flipResult);

    // Combine matches.
    std::vector<MatchResult> matches;
    if (FLAGS_reciprocal) {
      // Reduce to a consistent set.
      intersectionOfMatchResults(forward, reverse, matches);
      LOG(INFO) << "Found " << matches.size() << " reciprocal matches";
    } else {
      // Throw all matches together.
      unionOfMatchResults(forward, reverse, matches);
      LOG(INFO) << "Found " << matches.size() << " matches";
    }

    MatchResultWriter match_writer;
    ok = saveList(matches_file, matches, match_writer);
    CHECK(ok) << "Could not save list of matches";
  } else {
    std::vector<Match> forward;
    std::vector<Match> reverse;

    MatchReader match_reader;
    ok = loadList(forward_file, forward, match_reader);
    CHECK(ok) << "Could not load forward matches";
    LOG(INFO) << "Loaded " << forward.size() << " forward matches";
    ok = loadList(reverse_file, reverse, match_reader);
    CHECK(ok) << "Could not load reverse matches";
    LOG(INFO) << "Loaded " << reverse.size() << " reverse matches";

    // Flip order of reverse matches.
    std::transform(reverse.begin(), reverse.end(), reverse.begin(), flipMatch);

    // Combine matches.
    std::vector<Match> matches;
    if (FLAGS_reciprocal) {
      // Reduce to a consistent set.
      intersectionOfMatches(forward, reverse, matches);
      LOG(INFO) << "Found " << matches.size() << " reciprocal matches";
    } else if (FLAGS_directed_consistent) {
      forwardConsistentMatches(forward, reverse, matches);
      LOG(INFO) << "Found " << matches.size() << " forward-consistent matches";
    } else {
      // Throw all matches together.
      unionOfMatches(forward, reverse, matches);
      LOG(INFO) << "Found " << matches.size() << " matches";
    }

    MatchWriter match_writer;
    ok = saveList(matches_file, matches, match_writer);
    CHECK(ok) << "Could not save list of matches";
  }

  return 0;
}
