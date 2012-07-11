#include "keypoint.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>

////////////////////////////////////////////////////////////////////////////////
// Save keypoints to file.

namespace {

bool writeKeypointToFile(cv::FileStorage& out, const cv::KeyPoint& k) {
  // SIFT does not set 'response' or 'class_id' attributes.
  out << "{:";
  out << "x" << k.pt.x << "y" << k.pt.y;
  out << "size" << k.size;
  out << "angle" << k.angle;
  out << "octave" << k.octave;
  out << "}";

  return true;
}

}

bool saveKeypoints(const std::string& filename,
                   const std::vector<cv::KeyPoint>& keypoints) {
  // Open output file.
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    std::cerr << "could not open keypoints file" << std::endl;
    return 1;
  }

  file << "keypoints" << "[";

  // Write out keypoints.
  typedef std::vector<cv::KeyPoint> KeypointList;
  KeypointList::const_iterator keypoint;
  for (keypoint = keypoints.begin(); keypoint != keypoints.end(); ++keypoint) {
    writeKeypointToFile(file, *keypoint);
  }

  file << "]";

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Load keypoints from file.

namespace {

cv::KeyPoint readKeypointFromFileNode(const cv::FileNode& node) {
  cv::KeyPoint k;

  double x = (double)node["x"];
  double y = (double)node["y"];
  k.pt = cv::Point2d(x, y);
  k.size = (double)node["size"];
  k.angle = (double)node["angle"];
  k.octave = (double)node["octave"];

  return k;
}

}

bool loadKeypoints(const std::string& filename,
                   std::vector<cv::KeyPoint>& keypoints) {
  // Open file.
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }

  // Parse keypoints.
  cv::FileNode list = fs["keypoints"];
  std::transform(list.begin(), list.end(), std::back_inserter(keypoints),
      readKeypointFromFileNode);

  return true;
}

