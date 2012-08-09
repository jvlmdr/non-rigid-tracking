#ifndef NUMBER_WRITER_HPP_
#define NUMBER_WRITER_HPP_

#include "writer.hpp"

class NumberWriter : public Writer<double> {
  public:
    ~NumberWriter();
    void write(cv::FileStorage& file, const double& x);
};

#endif
