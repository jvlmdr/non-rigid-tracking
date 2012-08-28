#include "find_matches.hpp"
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// The result of a single match from one image to the other.
typedef cv::DMatch SingleDirectedMatchResult;
// A pair of single matches.
typedef std::vector<SingleDirectedMatchResult> SingleDirectedMatchResultPair;

void listToMatrix(const std::vector<Descriptor>& list, cv::Mat& matrix) {
  int num_descriptors = list.size();
  CHECK(!list.empty());
  int num_dimensions = list.front().data.size();
  matrix.create(num_descriptors, num_dimensions, cv::DataType<float>::type);

  for (int i = 0; i < num_descriptors; i += 1) {
    std::copy(list[i].data.begin(), list[i].data.end(),
        matrix.row(i).begin<float>());
  }
}

// Converts a pair of matches to a top-two match.
DirectedMatchResult pairToResult(const SingleDirectedMatchResultPair& pair) {
  DirectedMatchResult result;

  // Every element in the query set matched to one element in the training set.
  result.match = pair[0].trainIdx;
  result.dist = pair[0].distance;
  result.second_dist = pair[1].distance;

  return result;
}

void matchBothDirections(const std::vector<Descriptor>& list1,
                         const std::vector<Descriptor>& list2,
                         std::vector<DirectedMatchResult>& forward_matches,
                         std::vector<DirectedMatchResult>& reverse_matches,
                         bool use_flann) {
  // Load descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(list1, mat1);
  listToMatrix(list2, mat2);

  // Find two nearest neighbours, match in each direction.
  std::vector<SingleDirectedMatchResultPair> forward_match_pairs;
  std::vector<SingleDirectedMatchResultPair> reverse_match_pairs;

  // Construct matcher.
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (use_flann) {
    matcher = cv::DescriptorMatcher::create("FlannBased");
  } else {
    matcher = cv::DescriptorMatcher::create("BruteForce");
  }

  // Match all points in both directions.
  // Get top two matches as this is what we'll be doing in practice.
  matcher->knnMatch(mat1, mat2, forward_match_pairs, 2);
  matcher->knnMatch(mat2, mat1, reverse_match_pairs, 2);

  // Convert pairs to match results.
  forward_matches.clear();
  std::transform(forward_match_pairs.begin(), forward_match_pairs.end(),
      std::back_inserter(forward_matches), pairToResult);

  reverse_matches.clear();
  std::transform(reverse_match_pairs.begin(), reverse_match_pairs.end(),
      std::back_inserter(reverse_matches), pairToResult);
}

////////////////////////////////////////////////////////////////////////////////

MatchResult consistentMatchFromTwoDirected(const DirectedMatchResult& forward,
                                           const DirectedMatchResult& reverse) {
  MatchResult result;

  result.index1 = reverse.match;
  result.index2 = forward.match;
  result.dist = forward.dist;
  result.second_dist1 = forward.second_dist;
  result.second_dist2 = reverse.second_dist;

  return result;
}

void intersectionOfMatches(
    const std::vector<DirectedMatchResult>& forward_matches,
    const std::vector<DirectedMatchResult>& reverse_matches,
    std::vector<MatchResult>& matches) {
  matches.clear();

  // Have two lists of pairs (i, j).
  // Only keep pair (i, j) if there exists a pair (j, i) in the other list.
  // The first index i is unique in both lists.

  int num_forward_matches = forward_matches.size();

  for (int i = 0; i < num_forward_matches; i += 1) {
    // Get corresponding reverse match.
    const DirectedMatchResult& forward = forward_matches[i];
    const DirectedMatchResult& reverse = reverse_matches[forward.match];

    if (reverse.match == i) {
      // The reverse match points back. This match was consistent!
      matches.push_back(consistentMatchFromTwoDirected(forward, reverse));
    }
  }
}

void unionOfMatches(const std::vector<DirectedMatchResult>& forward_matches,
                    const std::vector<DirectedMatchResult>& reverse_matches,
                    std::vector<MatchResult>& matches) {
  matches.clear();

  // Have two lists of pairs (i, j).
  // Add all pairs from reverse list.
  // Only add (i, j) from forward list if (j, i) is not in reverse list.
  // The first index i is unique in both lists.

  int num_forward_matches = forward_matches.size();
  int num_reverse_matches = reverse_matches.size();

  for (int i = 0; i < num_reverse_matches; i += 1) {
    // Get corresponding reverse match.
    const DirectedMatchResult& reverse = reverse_matches[i];
    const DirectedMatchResult& forward = forward_matches[reverse.match];

    matches.push_back(consistentMatchFromTwoDirected(forward, reverse));
  }

  for (int i = 0; i < num_forward_matches; i += 1) {
    // Get corresponding reverse match.
    const DirectedMatchResult& forward = forward_matches[i];
    const DirectedMatchResult& reverse = reverse_matches[forward.match];

    if (reverse.match != i) {
      // The reverse match doesn't point back. This is a new match.
      matches.push_back(consistentMatchFromTwoDirected(forward, reverse));
    }
  }
}

