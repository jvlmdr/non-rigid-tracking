#ifndef WRITE_HPP_
#define WRITE_HPP_

#include <opencv2/core/core.hpp>

template<class T>
class Write {
  public:
    virtual ~Write() {}
    virtual void operator()(cv::FileStorage& file, const T& x) = 0;
};

#endif
