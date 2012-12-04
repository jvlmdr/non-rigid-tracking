#ifndef VIDEO_HPP_
#define VIDEO_HPP_

#include <opencv2/core/core.hpp>

class Video {
  public:
    virtual ~Video() {}

    // Accesses the image from time t.
    virtual bool get(int t, cv::Mat& image) const = 0;

    // Returns the number of frames in the video.
    virtual int length() const = 0;
};

#endif
