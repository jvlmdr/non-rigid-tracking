#include "match.hpp"

Match::Match() : first(-1), second(-1) {}

Match::Match(int first, int second) : first(first), second(second) {}

Match::Match(const std::pair<int, int>& pair)
    : first(pair.first), second(pair.second) {}

bool Match::operator<(const Match& other) const {
  if (first < other.first) {
    return true;
  } else if (other.first < first) {
    return false;
  } else {
    // first == other.first
    if (second < other.second) {
      return true;
    } else if (other.second < second) {
      return false;
    } else {
      // second == other.second
      // Strict inequality.
      return false;
    }
  }
}

std::pair<int, int> Match::pair() const {
  return std::pair<int, int>(first, second);
}

std::ostream& operator<<(std::ostream& stream, const Match& match) {
  return stream << "(" << match.first << ", " << match.second << ")";
}
