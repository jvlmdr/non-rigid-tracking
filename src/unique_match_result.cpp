#include "unique_match_result.hpp"
#include <limits>
#include <algorithm>

UniqueMatchResult::UniqueMatchResult()
    : index1(-1),
      index2(-1),
      distance(0),
      forward(false),
      reverse(false),
      next_best_forward(std::numeric_limits<double>::max()),
      next_best_reverse(std::numeric_limits<double>::max()) {}

UniqueMatchResult::UniqueMatchResult(int index1,
                                     int index2,
                                     double distance,
                                     double next_best_forward,
                                     double next_best_reverse)
    : index1(index1),
      index2(index2),
      distance(distance),
      forward(true),
      reverse(true),
      next_best_forward(next_best_forward),
      next_best_reverse(next_best_reverse) {}

UniqueMatchResult::UniqueMatchResult(int index1,
                                     int index2,
                                     double distance,
                                     bool forward,
                                     double next_best)
    : index1(index1),
      index2(index2),
      distance(distance),
      forward(forward),
      reverse(!forward),
      next_best_forward(std::numeric_limits<double>::max()),
      next_best_reverse(std::numeric_limits<double>::max()) {
  if (forward) {
    next_best_forward = next_best;
  } else {
    next_best_reverse = next_best;
  }
}

double UniqueMatchResult::minNextBest() const {
  double result = std::numeric_limits<double>::max();

  if (forward) {
    result = std::min(result, next_best_forward);
  }

  if (reverse) {
    result = std::min(result, next_best_reverse);
  }

  return result;
}
