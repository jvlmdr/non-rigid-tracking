#ifndef READ_HPP_
#define READ_HPP_

#include <opencv2/core/core.hpp>

template<class T>
class Read {
  public:
    virtual ~Read() {}
    virtual void operator()(const cv::FileNode& node, T& x) = 0;
};

#endif
