#ifndef MATCH_HPP_
#define MATCH_HPP_

#include <ostream>
#include <utility>

struct Match {
  int first;
  int second;

  Match();
  Match(int first, int second);
  explicit Match(const std::pair<int, int>& pair);

  bool operator<(const Match& other) const;

  std::pair<int, int> pair() const;
};

std::ostream& operator<<(std::ostream& stream, const Match& feature);

#endif
