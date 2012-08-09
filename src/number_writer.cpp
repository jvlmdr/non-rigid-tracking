#include "number_writer.hpp"

NumberWriter::~NumberWriter() {}

void NumberWriter::write(cv::FileStorage& file, const double& x) {
  file << "x" << x;
}
