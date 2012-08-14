#ifndef NUMBER_READER_HPP_
#define NUMBER_READER_HPP_

#include "reader.hpp"

class NumberReader : public Reader<double> {
  public:
    ~NumberReader();
    void read(const cv::FileNode& node, double& x);
};

#endif
