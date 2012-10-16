#include "match_result_reader.hpp"

MatchResultReader::~MatchResultReader() {}

bool MatchResultReader::read(const cv::FileNode& node, MatchResult& result) {
  if (!::read<int>(node["index1"], result.index1)) {
    return false;
  }

  if (!::read<int>(node["index2"], result.index2)) {
    return false;
  }

  if (!::read<double>(node["dist"], result.distance)) {
    return false;
  }

  return true;
}
