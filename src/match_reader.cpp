#include "match_reader.hpp"

MatchReader::~MatchReader() {}

bool MatchReader::read(const cv::FileNode& node, Match& match) {
  if (!::read<int>(node["index1"], match.first)) {
    return false;
  }

  if (!::read<int>(node["index2"], match.second)) {
    return false;
  }

  return true;
}
