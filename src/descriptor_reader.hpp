#ifndef DESCRIPTOR_READER_HPP_
#define DESCRIPTOR_READER_HPP_

#include "descriptor.hpp"
#include "reader.hpp"

class DescriptorReader : public Reader<Descriptor> {
  public:
    ~DescriptorReader();
    void read(const cv::FileNode& node, Descriptor& descriptor);
};

#endif
