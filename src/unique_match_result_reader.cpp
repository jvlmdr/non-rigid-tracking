#include "unique_match_result_reader.hpp"

UniqueMatchResultReader::~UniqueMatchResultReader() {}

bool UniqueMatchResultReader::read(const cv::FileNode& node,
                                   UniqueMatchResult& result) {
  if (!::read<int>(node["index1"], result.index1)) {
    return false;
  }

  if (!::read<int>(node["index2"], result.index2)) {
    return false;
  }

  if (!::read<double>(node["dist"], result.distance)) {
    return false;
  }

  // Attempt to read, but both might not be present.
  result.forward = ::read<double>(node["second_dist1"],
      result.next_best_forward);
  result.reverse = ::read<double>(node["second_dist2"],
      result.next_best_reverse);
  // Although at least one should be.
  if (!result.forward && !result.reverse) {
    return false;
  }

  return true;
}
