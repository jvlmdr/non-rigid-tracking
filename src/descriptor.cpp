#include "descriptor.hpp"
#include <iostream>
#include <stdexcept>
#include <opencv2/core/core.hpp>

////////////////////////////////////////////////////////////////////////////////
// Saving

namespace {

// Writes a single descriptor to a file.
bool writeDescriptor(cv::FileStorage& file, const Descriptor& descriptor) {
  file << "[:";

  for (int i = 0; i < descriptor.size(); i += 1) {
    file << descriptor[i];
  }

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

  std::vector<Descriptor>::const_iterator desc;
  for (desc = descriptors.begin(); desc != descriptors.end(); ++desc) {
    // Write descriptor to file.
    bool ok = writeDescriptor(file, *desc);

    if (!ok) {
      // Throw exception. It's unclear what to do after failing mid-write.
      throw std::runtime_error("could not write descriptor");
    }

    ++desc;
  }

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
