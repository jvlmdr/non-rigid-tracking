#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "read_image.hpp"
#include "keypoint.hpp"

const int MAX_NUM_FEATURES = 0;
const int NUM_OCTAVE_LAYERS = 3;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

DEFINE_double(contrast_threshold, 0.04,
    "Constrast threshold for feature detection");

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Finds SIFT keypoints in an image." << std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " image keypoints" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string image_filename = argv[1];
  std::string keypoints_filename = argv[2];

  double CONTRAST_THRESHOLD = FLAGS_contrast_threshold;

  // Read image.
  cv::Mat color_image;
  cv::Mat image;
  bool ok = readImage(image_filename, color_image, image);
  if (!ok) {
    std::cerr << "could not read image" << std::endl;
    return 1;
  }

  // Detect SIFT keypoints (scale-space maxima).
  std::vector<cv::KeyPoint> keypoints;
  cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, CONTRAST_THRESHOLD,
      EDGE_THRESHOLD, SIGMA);
  sift(image, cv::noArray(), keypoints, cv::noArray(), false);

  LOG(INFO) << "Found " << keypoints.size() << " keypoints";

  // Write out.
  ok = saveKeypoints(keypoints_filename, keypoints);

  return 0;
}
