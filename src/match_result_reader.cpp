#include "match_result_reader.hpp"

MatchResultReader::~MatchResultReader() {}

bool MatchResultReader::read(const cv::FileNode& node, MatchResult& result) {
  if (!::read<int>(node["index1"], result.index1)) {
    LOG(WARNING) << "Could not find `index1' parameter";
    return false;
  }

  if (!::read<int>(node["index2"], result.index2)) {
    LOG(WARNING) << "Could not find `index2' parameter";
    return false;
  }

  if (!::read<double>(node["dist"], result.distance)) {
    LOG(WARNING) << "Could not find `dist' parameter";
    return false;
  }

  return true;
}
