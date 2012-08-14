#include "descriptor_writer.hpp"
#include <boost/bind.hpp>

namespace {

template<typename T>
bool writeToFile(cv::FileStorage& file, const T& x) {
  file << x;
  return true;
}

}

DescriptorWriter::~DescriptorWriter() {}

void DescriptorWriter::write(cv::FileStorage& file,
                             const Descriptor& descriptor) {
  file << "list";
  file << "[:";
  std::for_each(descriptor.data.begin(), descriptor.data.end(),
      boost::bind(writeToFile<double>, boost::ref(file), _1));
  file << "]";
}
