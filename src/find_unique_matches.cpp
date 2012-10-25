#include "find_unique_matches.hpp"
#include "find_matches.hpp"
#include <map>
#include <set>
#include <algorithm>
#include <limits>
#include <utility>
#include <glog/logging.h>
#include <opencv2/features2d/features2d.hpp>
#include "find_matches_util.hpp"
#include "match_result.hpp"
#include "match.hpp"

namespace {

// A list of single matches.
typedef std::vector<cv::DMatch> RawMatchList;

typedef std::pair<double, double> DistancePair;

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

UniqueDirectedMatch findUniqueMatchUsingClassifier(
    const Classifier& classifier,
    const std::deque<Descriptor>& points) {
  // Find the top two matches for each classifier.
  std::vector<DirectedMatch> directed;
  findMatchesUsingClassifier(classifier, points, directed, 2);

  int index = directed[0].index;
  double distance = directed[0].distance;
  double next_best = directed[1].distance;

  return UniqueDirectedMatch(index, distance, next_best);
}

// Find the single best match for each point.
// Returns also the distance to the second-best match.
void findUniqueMatchesUsingClassifiers(
    const std::deque<Classifier>& classifiers,
    const std::deque<Descriptor>& points,
    std::vector<UniqueDirectedMatch>& matches) {
  matches.clear();

  // Evaluate the class
  std::deque<Classifier>::const_iterator classifier;
  for (classifier = classifiers.begin();
       classifier != classifiers.end();
       ++classifier) {
    matches.push_back(findUniqueMatchUsingClassifier(*classifier, points));
  }
}

////////////////////////////////////////////////////////////////////////////////

namespace {

void convertMatchPairs(const std::vector<RawMatchList>& pairs,
                       std::vector<UniqueDirectedMatch>& matches) {
  matches.clear();
  std::transform(pairs.begin(), pairs.end(), std::back_inserter(matches),
      convertMatchPair);
}

void findUniqueMatchesUsingEuclideanDistance(
    const cv::Mat& query,
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

void findUniqueMatchesInBothDirectionsUsingEuclideanDistance(
    const std::deque<Descriptor>& points1,
    const std::deque<Descriptor>& points2,
    std::vector<UniqueDirectedMatch>& forward,
    std::vector<UniqueDirectedMatch>& reverse,
    bool use_flann) {
  // Copy descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(points1, mat1);
  listToMatrix(points2, mat2);

  // Match forwards and backwards.
  findUniqueMatchesUsingEuclideanDistance(mat1, mat2, forward, use_flann);
  findUniqueMatchesUsingEuclideanDistance(mat2, mat1, reverse, use_flann);
}

////////////////////////////////////////////////////////////////////////////////

namespace {

// Returns a map of (index1, index2) -> (distance, next-best).
void addUniqueMatchesToSet(
    const std::map<int, UniqueDirectedMatch>& directed_matches,
    bool forward,
    std::map<Match, DistancePair>& undirected_matches) {
  undirected_matches.clear();

  std::map<int, UniqueDirectedMatch>::const_iterator iter;
  for (iter = directed_matches.begin();
       iter != directed_matches.end();
       ++iter) {
    int index1 = iter->first;
    const UniqueDirectedMatch& match = iter->second;
    int index2 = match.index;

    if (!forward) {
      std::swap(index1, index2);
    }

    std::pair<int, int> distances(match.distance, match.next_best);
    Match undirected_match(index1, index2);

    undirected_matches[undirected_match] = distances;
  }
}

}

void intersectionOfUniqueMatches(
    const std::map<int, UniqueDirectedMatch>& forward_matches,
    const std::map<int, UniqueDirectedMatch>& reverse_matches,
    std::vector<UniqueMatchResult>& matches) {
  // Construct a set of undirected matches from both sets of directed matches.
  typedef std::map<Match, DistancePair> Map;
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
      const Match& match = forward->first;

      //LOG(INFO) << forward->first << " " << reverse->first;

      double distance = forward->second.first; // == reverse->second.first
      double forward_next_best = forward->second.second;
      double reverse_next_best = reverse->second.second;

      UniqueMatchResult unique_match(match.first, match.second, distance,
          forward_next_best, reverse_next_best);
      matches.push_back(unique_match);

      ++forward;
      ++reverse;
    }
  }
}

void unionOfUniqueMatches(
    const std::map<int, UniqueDirectedMatch>& forward_matches,
    const std::map<int, UniqueDirectedMatch>& reverse_matches,
    std::vector<UniqueMatchResult>& matches) {
  // Construct a set of undirected matches from both sets of directed matches.
  typedef std::map<Match, DistancePair> Map;
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
    Match match;
    if (forward) {
      match = forward_match->first;
    } else {
      match = reverse_match->first;
    }

    UniqueMatchResult unique;
    if (reciprocal) {
      const DistancePair& forward_distances = forward_match->second;
      const DistancePair& reverse_distances = reverse_match->second;

      //LOG(INFO) << forward_match->first << " " << reverse_match->first;

      unique = UniqueMatchResult(match.first, match.second,
          forward_distances.first, forward_distances.second,
          reverse_distances.second);
    } else {
      // Match is only one-directional.
      if (forward) {
        const DistancePair& distances = forward_match->second;

        //LOG(INFO) << forward_match->first << " ()";

        unique = UniqueMatchResult(match.first, match.second, distances.first,
            true, distances.second);
      } else {
        const DistancePair& distances = reverse_match->second;

        //LOG(INFO) << "() " << reverse_match->first;

        unique = UniqueMatchResult(match.first, match.second, distances.first,
            false, distances.second);
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

void invert(const std::map<int, UniqueDirectedMatch>& matches,
            std::map<int, std::vector<UniqueDirectedMatch> >& map) {
  map.clear();

  std::map<int, UniqueDirectedMatch>::const_iterator iter;
  for (iter = matches.begin(); iter != matches.end(); ++iter) {
    int index = iter->first;
    const UniqueDirectedMatch& match = iter->second;

    UniqueDirectedMatch reverse(index, match.distance, match.next_best);
    map[match.index].push_back(reverse);
  }
}

void forwardConsistentUniqueMatches(
    const std::map<int, UniqueDirectedMatch>& forward_matches,
    const std::map<int, UniqueDirectedMatch>& reverse_matches,
    std::vector<UniqueMatchResult>& undirected_matches) {
  undirected_matches.clear();

  // Group forward matches by their feature in the second image.
  std::map<int, std::vector<UniqueDirectedMatch> > map;
  invert(forward_matches, map);

  // The set of features in the first image which have been matched.
  std::set<int> forward_matched;

  // Add all forward matches which do not share a feature in the second image.
  {
    std::map<int, std::vector<UniqueDirectedMatch> >::const_iterator iter;
    for (iter = map.begin(); iter != map.end(); ++iter) {
      int index = iter->first;
      const std::vector<UniqueDirectedMatch>& matches = iter->second;

      if (matches.size() == 1) {
        const UniqueDirectedMatch& reverse = matches.front();
        // Add match to the list.
        UniqueMatchResult match(reverse.index, index, reverse.distance, true,
            reverse.next_best);
        undirected_matches.push_back(match);
        // Mark this feature as matched.
        forward_matched.insert(reverse.index);
      }
    }
  }

  // Add all reverse matches for unmatched features in the first image.
  {
    std::map<int, UniqueDirectedMatch>::const_iterator iter;
    for (iter = reverse_matches.begin();
         iter != reverse_matches.end();
         ++iter) {
      int index = iter->first;
      const UniqueDirectedMatch& reverse = iter->second;

      // Check that the feature in the first image has not been matched yet.
      if (forward_matched.count(reverse.index) == 0) {
        UniqueMatchResult match(reverse.index, index, reverse.distance, false,
            reverse.next_best);
        undirected_matches.push_back(match);
      }
    }
  }
}
