#include "descriptor_sequence.hpp"
#include <iostream>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>

namespace {

template<typename T>
bool writeToFile(cv::FileStorage& file, const T& x) {
  file << x;
  return true;
}

// Writes a single descriptor to a file.
bool writeIndexedDescriptor(cv::FileStorage& file,
                            const IndexedDescriptor& indexed) {
  int t = indexed.first;
  const Descriptor& descriptor = indexed.second;

  file << "{";
  file << "t" << t;
  file << "descriptor" << "[:";
  std::for_each(descriptor.begin(), descriptor.end(),
      boost::bind(writeToFile<double>, boost::ref(file), _1));
  file << "]";
  file << "}";

  return true;
}

// Writes a single descriptor to a file.
bool writeDescriptorSequence(cv::FileStorage& file,
                             const DescriptorSequence& sequence) {
  file << "[";
  std::for_each(sequence.begin(), sequence.end(),
      boost::bind(writeIndexedDescriptor, boost::ref(file), _1));
  file << "]";

  return true;
}

}

bool saveDescriptorSequences(const std::string& filename,
                             const DescriptorSequenceList& sequences) {
  // Open file.
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    std::cerr << "could not open file " << filename << std::endl;
    return false;
  }

  file << "tracks" << "[";
  std::for_each(sequences.begin(), sequences.end(),
      boost::bind(writeDescriptorSequence, boost::ref(file), _1));
  file << "]";

  return true;
}
