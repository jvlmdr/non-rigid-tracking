#include "match_writer.hpp"

MatchWriter::~MatchWriter() {}

void MatchWriter::write(cv::FileStorage& file, const Match& match) {
  file << "index1" << match.index1;
  file << "index2" << match.index2;
}
