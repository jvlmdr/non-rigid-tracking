#include "number_reader.hpp"

NumberReader::~NumberReader() {}

void NumberReader::read(const cv::FileNode& node, double& x) {
  x = static_cast<double>(node["x"]);
}
