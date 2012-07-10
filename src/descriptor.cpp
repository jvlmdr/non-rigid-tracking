#include "descriptor.hpp"
#include <iostream>
#include <stdexcept>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>

////////////////////////////////////////////////////////////////////////////////
// Saving

namespace {

template<typename T>
bool writeToFile(cv::FileStorage& file, const T& x) {
  file << x;
  return true;
}

// Writes a single descriptor to a file.
bool writeDescriptor(cv::FileStorage& file, const Descriptor& descriptor) {
  file << "[:";
  std::for_each(descriptor.begin(), descriptor.end(),
      boost::bind(writeToFile<double>, boost::ref(file), _1));
  file << "]";

  return true;
}

}

// Saves a list of descriptors to a file.
bool saveDescriptors(const std::string& filename,
                     const std::vector<Descriptor>& descriptors) {
  // Open file to save tracks.
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    std::cerr << "could not open file " << filename << std::endl;
    return false;
  }

  file << "descriptors" << "[";
  std::for_each(descriptors.begin(), descriptors.end(),
      boost::bind(writeDescriptor, boost::ref(file), _1));
  file << "]";

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Loading

namespace {

template<class Input, class Output>
Output cast(const Input& x) {
  return static_cast<Output>(x);
}

// Writes a single descriptor to a file.
Descriptor readDescriptor(const cv::FileNode& node) {
  Descriptor descriptor;
  std::transform(node.begin(), node.end(), std::back_inserter(descriptor),
      cast<const cv::FileNode&, double>);
  return descriptor;
}

}

// Loads a list of descriptors from a file.
bool loadDescriptors(const std::string& filename,
                     std::vector<Descriptor>& descriptors) {
  // Open file.
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }

  // Parse keypoints.
  cv::FileNode list = fs["descriptors"];
  std::transform(list.begin(), list.end(), std::back_inserter(descriptors),
      readDescriptor);

  return true;
}
