#include "find_unique_matches.hpp"
#include <map>
#include <glog/logging.h>
#include <opencv2/features2d/features2d.hpp>
#include "find_matches_util.hpp"
#include "match_result.hpp"

namespace {

// A list of single matches.
typedef std::vector<cv::DMatch> RawMatchList;

}

////////////////////////////////////////////////////////////////////////////////

UniqueDirectedMatch::UniqueDirectedMatch()
    : index(-1), distance(0), next_best(0) {}

UniqueDirectedMatch::UniqueDirectedMatch(int index,
                                         double distance,
                                         double next_best)
    : index(index), distance(distance), next_best(next_best) {}

UniqueDirectedMatch convertMatchPair(const RawMatchList& pair) {
  CHECK(pair.size() == 2);

  int index = pair[0].trainIdx;
  double distance = pair[0].distance;
  double next_best = pair[1].distance;

  return UniqueDirectedMatch(index, distance, next_best);
}

////////////////////////////////////////////////////////////////////////////////

namespace {

void convertMatchPairs(const std::vector<RawMatchList>& pairs,
                       std::vector<UniqueDirectedMatch>& matches) {
  matches.clear();
  std::transform(pairs.begin(), pairs.end(), std::back_inserter(matches),
      convertMatchPair);
}

void matchUnique(const cv::Mat& query,
                 const cv::Mat& train,
                 std::vector<UniqueDirectedMatch>& matches,
                 bool use_flann) {
  // Construct matcher.
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (use_flann) {
    matcher = cv::DescriptorMatcher::create("FlannBased");
  } else {
    matcher = cv::DescriptorMatcher::create("BruteForce");
  }

  // Take top two matches.
  std::vector<std::vector<cv::DMatch> > raw;
  matcher->knnMatch(query, train, raw, 2);

  // Convert from cv::DMatch to UniqueMatchResult.
  convertMatchPairs(raw, matches);
}

}

void matchUniqueBothDirections(
    const std::vector<Descriptor>& points1,
    const std::vector<Descriptor>& points2,
    std::vector<UniqueDirectedMatch>& forward,
    std::vector<UniqueDirectedMatch>& reverse,
    bool use_flann) {
  // Copy descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(points1, mat1);
  listToMatrix(points2, mat2);

  // Match forwards and backwards.
  matchUnique(mat1, mat2, forward, use_flann);
  matchUnique(mat2, mat1, reverse, use_flann);
}

////////////////////////////////////////////////////////////////////////////////

namespace {

// Returns a map of (index1, index2, distance) -> second-distance because the
// second-best distance, unlike the distance, will be different for forward and
// reverse matches.
void addUniqueMatchesToSet(const std::vector<UniqueDirectedMatch>& directed,
                           bool forward,
                           std::map<MatchResult, double>& undirected) {
  std::vector<UniqueDirectedMatch>::const_iterator d;
  int index = 0;

  for (d = directed.begin(); d != directed.end(); ++d) {
    int index1 = index;
    int index2 = d->index;

    if (!forward) {
      std::swap(index1, index2);
    }

    MatchResult u(index1, index2, d->distance);
    undirected[u] = d->next_best;

    index += 1;
  }
}

}

void intersectionOfUniqueMatches(
    const std::vector<UniqueDirectedMatch>& forward_matches,
    const std::vector<UniqueDirectedMatch>& reverse_matches,
    std::vector<UniqueMatchResult>& matches) {
  // Construct a set of undirected matches from both sets of directed matches.
  typedef std::map<MatchResult, double> Map;
  Map forward_set;
  Map reverse_set;
  addUniqueMatchesToSet(forward_matches, true, forward_set);
  addUniqueMatchesToSet(reverse_matches, false, reverse_set);

  matches.clear();

  // Do a std::set_intersection but keep both second-best distances.
  Map::const_iterator forward = forward_set.begin();
  Map::const_iterator reverse = reverse_set.begin();

  while (forward != forward_set.end() && reverse != reverse_set.end()) {
    if (forward->first < reverse->first) {
      ++forward;
    } else if (reverse->first < forward->first) {
      ++reverse;
    } else {
      // Found a reciprocal match.
      const MatchResult& match = forward->first;

      UniqueMatchResult unique_match(match.index1, match.index2, match.distance,
          forward->second, reverse->second);
      matches.push_back(unique_match);

      ++forward;
      ++reverse;
    }
  }
}

void unionOfUniqueMatches(
    const std::vector<UniqueDirectedMatch>& forward_matches,
    const std::vector<UniqueDirectedMatch>& reverse_matches,
    std::vector<UniqueMatchResult>& matches) {
  // Construct a set of undirected matches from both sets of directed matches.
  typedef std::map<MatchResult, double> Map;
  Map forward_set;
  Map reverse_set;
  addUniqueMatchesToSet(forward_matches, true, forward_set);
  addUniqueMatchesToSet(reverse_matches, false, reverse_set);

  matches.clear();

  // Do a std::set_intersection but keep both second-best distances.
  Map::const_iterator forward_match = forward_set.begin();
  Map::const_iterator reverse_match = reverse_set.begin();

  while (forward_match != forward_set.end() ||
      reverse_match != reverse_set.end()) {
    bool reciprocal;
    bool forward;

    if (forward_match == forward_set.end()) {
      // Only reverse matches remain.
      reciprocal = false;
      forward = false;
    } else if (reverse_match == reverse_set.end()) {
      // Only forward matches remain.
      reciprocal = false;
      forward = true;
    } else {
      // Neither iterator has reached the end.
      if (forward_match->first < reverse_match->first) {
        reciprocal = false;
        forward = true;
      } else if (reverse_match->first < forward_match->first) {
        reciprocal = false;
        forward = false;
      } else {
        reciprocal = true;
        forward = true; // irrelevant
      }
    }

    // If match is reciprocal, it doesn't matter which we use.
    MatchResult match;
    if (forward) {
      match = forward_match->first;
    } else {
      match = reverse_match->first;
    }

    UniqueMatchResult unique;
    if (reciprocal) {
      unique = UniqueMatchResult(match.index1, match.index2, match.distance,
          forward_match->second, reverse_match->second);
    } else {
      // Match is only one-directional.
      if (forward) {
        unique = UniqueMatchResult(match.index1, match.index2, match.distance,
            true, forward_match->second);
      } else {
        unique = UniqueMatchResult(match.index1, match.index2, match.distance,
            false, reverse_match->second);
      }
    }
    matches.push_back(unique);

    // If reciprocal, advance both. Otherwise just one.
    if (reciprocal) {
      ++forward_match;
      ++reverse_match;
    } else {
      if (forward) {
        ++forward_match;
      } else {
        ++reverse_match;
      }
    }
  }
}
