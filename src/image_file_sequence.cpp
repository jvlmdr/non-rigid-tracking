#include "image_file_sequence.hpp"
#include <glog/logging.h>
#include "read_image.hpp"
#include "util.hpp"

ImageFileSequence::ImageFileSequence(const boost::format& format,
                                     int length,
                                     bool gray)
    : format_(format), length_(length), gray_(gray) {}

ImageFileSequence::ImageFileSequence(const boost::format& format, bool gray)
    : format_(format), length_(0), gray_(gray) {}

ImageFileSequence::~ImageFileSequence() {}

int ImageFileSequence::countFrames() {
  length_ = 0;

  while (fileExists(makeFilename(length_))) {
    length_ += 1;
  }
  LOG(INFO) << "Found " << length_ << " frames";

  return length_;
}

bool ImageFileSequence::get(int t, cv::Mat& image) const {
  bool ok;

  if (gray_) {
    ok = readGrayImage(makeFilename(t), image);
  } else {
    ok = readColorImage(makeFilename(t), image);
  }

  return ok;
}

int ImageFileSequence::length() const {
  return length_;
}

std::string ImageFileSequence::makeFilename(int t) const {
  return boost::str(boost::format(format_) % (t + 1));
}
