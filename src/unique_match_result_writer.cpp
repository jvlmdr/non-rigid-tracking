#include "unique_match_result_writer.hpp"

UniqueMatchResultWriter::~UniqueMatchResultWriter() {}

void UniqueMatchResultWriter::write(cv::FileStorage& file,
                                    const UniqueMatchResult& result) {
  file << "index1" << result.index1;
  file << "index2" << result.index2;
  file << "dist" << result.distance;
  if (result.forward) {
    file << "second_dist1" << result.next_best_forward;
  }
  if (result.reverse) {
    file << "second_dist2" << result.next_best_reverse;
  }
}
