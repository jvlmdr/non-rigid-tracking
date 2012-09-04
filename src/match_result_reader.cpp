#include "match_result_reader.hpp"

MatchResultReader::~MatchResultReader() {}

bool MatchResultReader::read(const cv::FileNode& node, MatchResult& result) {
  if (!::read<int>(node["index1"], result.index1)) {
    return false;
  }

  if (!::read<int>(node["index2"], result.index2)) {
    return false;
  }

  if (!::read<double>(node["dist"], result.dist)) {
    return false;
  }

  if (!::read<double>(node["second_dist1"], result.second_dist1)) {
    return false;
  }

  if (!::read<double>(node["second_dist2"], result.second_dist2)) {
    return false;
  }

  return true;
}
