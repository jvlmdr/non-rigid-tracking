#include "feature_index.hpp"

FeatureIndex::FeatureIndex() : view(-1), time(-1), id(-1) {}

FeatureIndex::FeatureIndex(int view, int time, int id)
    : view(view), time(time), id(id) {}

FeatureIndex::FeatureIndex(const Frame& frame, int id)
    : view(frame.view), time(frame.time), id(id) {}

bool FeatureIndex::operator<(const FeatureIndex& other) const {
  if (view < other.view) {
    return true;
  } else if (other.view < view) {
    return false;
  } else {
    if (time < other.time) {
      return true;
    } else if (other.time < time) {
      return false;
    } else {
      return id < other.id;
    }
  }
}

std::ostream& operator<<(std::ostream& stream, const FeatureIndex& feature) {
  return stream << "(" << feature.view << ", " << feature.time << ", " <<
      feature.id << ")";
}
