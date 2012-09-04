#include "descriptor_reader.hpp"
#include "vector_reader.hpp"
#include "default_reader.hpp"

DescriptorReader::~DescriptorReader() {}

bool DescriptorReader::read(const cv::FileNode& node, Descriptor& descriptor) {
  DefaultReader<double> number_reader;
  VectorReader<double> reader(number_reader);
  return reader.read(node["list"], descriptor.data);
}
