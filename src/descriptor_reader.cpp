#include "descriptor_reader.hpp"
#include "iterator_reader.hpp"
#include "default_reader.hpp"

DescriptorReader::~DescriptorReader() {}

bool DescriptorReader::read(const cv::FileNode& node, Descriptor& descriptor) {
  descriptor.clear();
  InlineDefaultReader<double> reader;
  return readSequence(node, reader, std::back_inserter(descriptor.data));
}
