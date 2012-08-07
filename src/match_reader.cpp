#include "match_reader.hpp"

MatchReader::~MatchReader() {}

void MatchReader::read(const cv::FileNode& node, Match& match) {
  match.first = static_cast<int>(node["index1"]);
  match.second = static_cast<int>(node["index2"]);
}
