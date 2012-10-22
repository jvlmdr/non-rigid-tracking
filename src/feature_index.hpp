#ifndef FEATURE_INDEX_HPP_
#define FEATURE_INDEX_HPP_

#include <ostream>
#include "multiview_track.hpp"

// Identifies a feature in a multiview video.
// TODO: Need a better name?
struct FeatureIndex {
  int view;
  int time;
  int id;

  FeatureIndex();
  FeatureIndex(int view, int time, int id);
  FeatureIndex(const Frame& frame, int id);

  // Defines an ordering over feature indices.
  bool operator<(const FeatureIndex& other) const;
};

std::ostream& operator<<(std::ostream& stream, const FeatureIndex& feature);

#endif
