#include "match_result.hpp"

MatchResult::MatchResult() : index1(-1), index2(-1), distance(-1) {}

MatchResult::MatchResult(int index1, int index2, double distance)
    : index1(index1), index2(index2), distance(distance) {}

MatchResult::MatchResult(const Match& match, double distance)
    : index1(match.first), index2(match.second), distance(distance) {}

bool MatchResult::operator==(const MatchResult& other) const {
  return index1 == other.index1 && index2 == other.index2 &&
      distance == other.distance;
}
