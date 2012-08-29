#include "find_matches.hpp"
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// The result of a single match from one image to the other.
typedef cv::DMatch SingleDirectedMatchResult;
// A list of single matches.
typedef std::vector<SingleDirectedMatchResult> SingleDirectedMatchResultList;

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

void bagsToMatrices(const std::vector<DescriptorBag>& bags,
                    std::vector<cv::Mat>& matrices) {
  int num_bags = bags.size();
  // Initialize list of matrices.
  matrices.assign(num_bags, cv::Mat());
  // Copy lists of descriptors into matrices.
  for (int i = 0; i < num_bags; i += 1) {
    listToMatrix(bags[i], matrices[i]);
  }
}

// Converts a pair of matches to a top-two match.
DirectedMatchResult pairToResult(const SingleDirectedMatchResultList& pair) {
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
  std::vector<SingleDirectedMatchResultList> forward_match_pairs;
  std::vector<SingleDirectedMatchResultList> reverse_match_pairs;

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

////////////////////////////////////////////////////////////////////////////////

int findBestMatch(const std::vector<SingleDirectedMatchResultList>& list) {
  CHECK(list.size() > 0);
  int n = list.size();
  int arg = 0;

  for (int i = 1; i < n; i += 1) {
    if (list[i].front() < list[arg].front()) {
      arg = i;
    }
  }

  return arg;
}

int sizeOfLargestBag(const std::vector<cv::Mat>& matrices) {
  int max = 0;
  int n = matrices.size();

  for (int i = 0; i < n; i += 1) {
    max = std::max(matrices[i].rows, max);
  }

  return max;
}

void flattenMatches(const std::vector<SingleDirectedMatchResultList>& matches,
                    SingleDirectedMatchResultList& flat) {
  int n = matches.size();
  flat.clear();

  for (int i = 0; i < n; i += 1) {
    std::copy(matches[i].begin(), matches[i].end(), std::back_inserter(flat));
  }
}

void matchBags(const std::vector<cv::Mat>& query,
               const std::vector<cv::Mat>& train,
               std::vector<DirectedMatchResult>& results,
               bool use_flann) {
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (use_flann) {
    matcher = cv::DescriptorMatcher::create("FlannBased");
  } else {
    matcher = cv::DescriptorMatcher::create("BruteForce");
  }

  int num_query_features = query.size();
  results.clear();

  // Find a match in the train image for every feature in the query image.
  matcher->add(train);
  // Find largest bag in train image.
  int largest = sizeOfLargestBag(train);

  for (int i = 0; i < num_query_features; i += 1) {
    int num_queries = query[i].rows;
    CHECK(num_queries > 0);

    // Find best match for each feature in the bag.
    std::vector<SingleDirectedMatchResultList> grouped_matches;
    matcher->knnMatch(query[i], grouped_matches, largest + 1);

    // Flatten out into a single list.
    SingleDirectedMatchResultList matches;
    flattenMatches(grouped_matches, matches);

    // Sort all of them. (This could be more efficient!)
    std::sort(matches.begin(), matches.end());

    // Read best match from front of list.
    DirectedMatchResult result;
    result.match = matches.front().imgIdx;
    result.dist = matches.front().distance;

    // Find first match belonging to a different feature.
    int j = 0;
    int num_matches = matches.size();
    while (matches[j].imgIdx == result.match) {
      j += 1;
      CHECK(j < num_matches);
    }
    result.second_dist = matches[j].distance;

    results.push_back(result);
  }
}

void matchBagsBothDirections(const std::vector<DescriptorBag>& bags1,
                             const std::vector<DescriptorBag>& bags2,
                             std::vector<DirectedMatchResult>& forward_matches,
                             std::vector<DirectedMatchResult>& reverse_matches,
                             bool use_flann) {
  // Load descriptors into matrices for cv::DescriptorMatcher.
  std::vector<cv::Mat> matrices1;
  std::vector<cv::Mat> matrices2;
  bagsToMatrices(bags1, matrices1);
  bagsToMatrices(bags2, matrices2);

  // Find a match in the second image for each feature in the first image...
  matchBags(matrices1, matrices2, forward_matches, use_flann);
  // ...and vice versa.
  matchBags(matrices2, matrices1, reverse_matches, use_flann);
}
