#include "match_result_reader.hpp"

MatchResultReader::~MatchResultReader() {}

void MatchResultReader::read(const cv::FileNode& node, MatchResult& result) {
  result.index1 = static_cast<int>(node["index1"]);
  result.index2 = static_cast<int>(node["index2"]);
  result.dist = static_cast<double>(node["dist"]);
  result.second_dist1 = static_cast<double>(node["second_dist1"]);
  result.second_dist2 = static_cast<double>(node["second_dist2"]);
}
