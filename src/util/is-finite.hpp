#ifndef UTIL_IS_FINITE_HPP_
#define UTIL_IS_FINITE_HPP_

inline bool isFinite(double x) {
  return !(std::numeric_limits<double>::max() < x ||
           x < -std::numeric_limits<double>::max());
}

#endif
