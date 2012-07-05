#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "read_image.hpp"

// Maximum number of features. 0 for no limit.
const int MAX_NUM_FEATURES = 0;

bool writeKeyPointToFile(cv::FileStorage& out, const cv::KeyPoint& k) {
  // SIFT does not set 'response' or 'class_id' attributes.
  out << "{:";
  out << "x" << k.pt.x << "y" << k.pt.y;
  out << "size" << k.size;
  out << "angle" << k.angle;
  out << "octave" << k.octave;
  out << "}";
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " image keypoints"
              << std::endl;
    std::cerr << std::endl;
    std::cerr << "Example" << std::endl;
    std::cerr << argv[0]
              << " input/my-video/005.png output/my-video/keypoints-005.yaml"
              << std::endl;
    return 1;
  }

  std::string image_filename = argv[1];
  std::string keypoints_filename = argv[2];

  // Read image.
  cv::Mat color_image;
  cv::Mat image;
  bool ok = readImage(image_filename, color_image, image);
  if (!ok) {
    std::cerr << "could not read image" << std::endl;
    return 1;
  }

  // Open output file.
  cv::FileStorage keypoints_file(keypoints_filename, cv::FileStorage::WRITE);
  if (!keypoints_file.isOpened()) {
    std::cerr << "could not open keypoints file" << std::endl;
  }

  // Detect SIFT keypoints (scale-space maxima).
  typedef std::vector<cv::KeyPoint> KeyPointList;
  KeyPointList keypoints;
  cv::SIFT sift(MAX_NUM_FEATURES);
  sift(image, cv::noArray(), keypoints, cv::noArray(), false);

  keypoints_file << "keypoints" << "[";

  // Write out keypoints.
  KeyPointList::const_iterator keypoint;
  for (keypoint = keypoints.begin(); keypoint != keypoints.end(); ++keypoint) {
    writeKeyPointToFile(keypoints_file, *keypoint);
  }

  keypoints_file << "]";

  return 0;
}
