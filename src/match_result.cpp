#include "match_result.hpp"

MatchResult::MatchResult() : index1(-1), index2(-1), distance(-1) {}

MatchResult::MatchResult(int index1, int index2, double distance)
    : index1(index1), index2(index2), distance(distance) {}

bool MatchResult::operator<(const MatchResult& other) const {
  if (index1 < other.index1) {
    return true;
  } else if (other.index1 < index1) {
    return false;
  } else {
    // index1 == other.index1
    if (index2 < other.index2) {
      return true;
    } else if (other.index2 < index2) {
      return false;
    } else {
      // index2 == other.index2
      // Strict inequality.
      return false;
    }
  }
}
