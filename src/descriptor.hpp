#ifndef DESCRIPTOR_HPP_
#define DESCRIPTOR_HPP_

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include "read.hpp"
#include "write.hpp"
#include "reader.hpp"

struct Descriptor {
  typedef std::vector<double> Data;
  Data data;

  // Writes a single descriptor to a file.
  void write(cv::FileStorage& file) const;

  // Reads a descriptor from a file.
  void read(const cv::FileNode& node);
};

struct WriteDescriptor : public Write<Descriptor> {
  ~WriteDescriptor();
  void operator()(cv::FileStorage& file, const Descriptor& descriptor);
};

struct ReadDescriptor : public Read<Descriptor> {
  ~ReadDescriptor();
  void operator()(const cv::FileNode& node, Descriptor& descriptor);
};

class DescriptorReader : public Reader<Descriptor> {
  public:
    ~DescriptorReader();
    void read(const cv::FileNode& node, Descriptor& descriptor);
};

// Saves a list of descriptors to a file.
bool saveDescriptors(const std::string& filename,
                     const std::vector<Descriptor>& descriptors);

// Loads a list of descriptors from a file.
bool loadDescriptors(const std::string& filename,
                     std::vector<Descriptor>& descriptors);

#endif
