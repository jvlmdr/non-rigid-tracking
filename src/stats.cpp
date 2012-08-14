#include "stats.hpp"
#include <numeric>
#include <algorithm>

double computeMean(const std::vector<double>& x) {
  return std::accumulate(x.begin(), x.end(), 0.) / x.size();
}

double computeMedian(const std::vector<double>& x) {
  if (x.empty()) {
    return 0. / 0.;
  }

  std::vector<double> y;
  std::copy(x.begin(), x.end(), std::back_inserter(y));
  std::sort(y.begin(), y.end());
  return y[x.size() / 2];
}

int countLessThanEqualTo(const std::vector<double>& x, double a) {
  int n = 0;

  std::vector<double>::const_iterator it;
  for (it = x.begin(); it != x.end(); ++it) {
    if (*it <= a) {
      n += 1;
    }
  }

  return n;
}
