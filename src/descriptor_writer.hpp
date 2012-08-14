#ifndef DESCRIPTOR_WRITER_HPP_
#define DESCRIPTOR_WRITER_HPP_

#include "descriptor.hpp"
#include "writer.hpp"

class DescriptorWriter : public Writer<Descriptor> {
  public:
    ~DescriptorWriter();
    void write(cv::FileStorage& file, const Descriptor& descriptor);
};

#endif
