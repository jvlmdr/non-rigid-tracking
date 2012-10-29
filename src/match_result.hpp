#ifndef MATCH_RESULT_HPP_
#define MATCH_RESULT_HPP_

#include "match.hpp"

// A MatchResult (compared to a simple Match) includes the distance.
struct MatchResult {
  int index1;
  int index2;
  double distance;

  MatchResult();
  MatchResult(int index1, int index2, double distance);
  MatchResult(const Match& match, double distance);

  bool operator==(const MatchResult& other) const;
};

#endif
