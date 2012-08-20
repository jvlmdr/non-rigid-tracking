#include "match_result_writer.hpp"

MatchResultWriter::~MatchResultWriter() {}

void MatchResultWriter::write(cv::FileStorage& file,
                              const MatchResult& result) {
  file << "index1" << result.index1;
  file << "index2" << result.index2;
  file << "dist" << result.dist;
  file << "second_dist1" << result.second_dist1;
  file << "second_dist2" << result.second_dist2;
}
