#include "find_matches_util.hpp"
#include <algorithm>
#include <glog/logging.h>

// Copies a list of descriptors into a matrix of descriptor rows.
void listToMatrix(const std::deque<Descriptor>& list, cv::Mat& matrix) {
  int num_descriptors = list.size();
  CHECK(!list.empty());
  int num_dimensions = list.front().data.size();
  matrix.create(num_descriptors, num_dimensions, cv::DataType<float>::type);

  for (int i = 0; i < num_descriptors; i += 1) {
    cv::Mat row = matrix.row(i);
    std::copy(list[i].data.begin(), list[i].data.end(), row.begin<float>());
  }
}
