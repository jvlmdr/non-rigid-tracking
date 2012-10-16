#include "match_result_writer.hpp"

MatchResultWriter::~MatchResultWriter() {}

void MatchResultWriter::write(cv::FileStorage& file,
                              const MatchResult& result) {
  file << "index1" << result.index1;
  file << "index2" << result.index2;
  file << "dist" << result.distance;
}
