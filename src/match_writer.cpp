#include "match_writer.hpp"

MatchWriter::~MatchWriter() {}

void MatchWriter::write(cv::FileStorage& file, const Match& match) {
  file << "index1" << match.first;
  file << "index2" << match.second;
}
