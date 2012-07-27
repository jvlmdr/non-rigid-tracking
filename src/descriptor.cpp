#include "descriptor.hpp"
#include <iostream>
#include <stdexcept>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>

WriteDescriptor::~WriteDescriptor() {}

void WriteDescriptor::operator()(cv::FileStorage& file,
                                 const Descriptor& descriptor) {
  file << "{";
  file << "descriptor";
  descriptor.write(file);
  file << "}";
}

ReadDescriptor::~ReadDescriptor() {}

void ReadDescriptor::operator()(const cv::FileNode& node,
                                Descriptor& descriptor) {
  descriptor.read(node["descriptor"]);
}

namespace {

template<typename T>
bool writeToFile(cv::FileStorage& file, const T& x) {
  file << x;
  return true;
}

template<class Input, class Output>
Output cast(const Input& x) {
  return static_cast<Output>(x);
}

}

void Descriptor::write(cv::FileStorage& file) const {
  file << "[:";
  std::for_each(data.begin(), data.end(),
      boost::bind(writeToFile<double>, boost::ref(file), _1));
  file << "]";
}

void Descriptor::read(const cv::FileNode& node) {
  data.clear();
  std::transform(node.begin(), node.end(), std::back_inserter(data),
      cast<const cv::FileNode&, double>);
}

////////////////////////////////////////////////////////////////////////////////
// Saving


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
      boost::bind(&Descriptor::write, _1, boost::ref(file)));
  file << "]";

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Loading

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
  cv::FileNodeIterator node;
  for (node = list.begin(); node != list.end(); ++node) {
    descriptors.push_back(Descriptor());
    descriptors.back().read(*node);
  }

  return true;
}
