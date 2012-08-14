#include "descriptor_reader.hpp"

namespace {

template<class Input, class Output>
Output cast(const Input& x) {
  return static_cast<Output>(x);
}

}

DescriptorReader::~DescriptorReader() {}

void DescriptorReader::read(const cv::FileNode& node, Descriptor& descriptor) {
  const cv::FileNode& list = node["list"];

  descriptor.data.clear();
  std::transform(list.begin(), list.end(), std::back_inserter(descriptor.data),
      cast<const cv::FileNode&, double>);
}
