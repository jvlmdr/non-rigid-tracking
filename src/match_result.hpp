#ifndef MATCH_RESULT_HPP_
#define MATCH_RESULT_HPP_

// The result of a consistent match.
//
// Contains:
// Indices of matching features.
// Distance between matching features.
// Distance to each feature's second-nearest match.
struct MatchResult {
  int index1;
  int index2;
  double dist;
  double second_dist1;
  double second_dist2;
};

#endif
