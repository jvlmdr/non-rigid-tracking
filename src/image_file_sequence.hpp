#ifndef IMAGE_FILE_SEQUENCE_HPP_
#define IMAGE_FILE_SEQUENCE_HPP_

#include "video.hpp"
#include <boost/format.hpp>

// Describes a video saved as a sequence of frame images.
class ImageFileSequence : public Video {
  public:
    ImageFileSequence(const boost::format& format, int length, bool gray);

    // This constructor sets the number of frames to zero.
    // Follow it with a call to countFrames().
    ImageFileSequence(const boost::format& format, bool gray);

    ~ImageFileSequence();

    // Counts the number of frames based on the existence of image files.
    int countFrames();

    // Accesses the image from time t.
    bool get(int t, cv::Mat& image) const;

    // Returns the number of frames in the video.
    int length() const;

  private:
    // Returns the filename for a frame.
    std::string makeFilename(int t) const;

    boost::format format_;
    int length_;
    bool gray_;
};

#endif
