#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cstdlib>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <gflags/gflags.h>
#include "read_image.hpp"
#include "descriptor.hpp"
#include "keypoint.hpp"
#include "match.hpp"
#include "random_color.hpp"
#include "rigid_feature.hpp"
#include "rigid_warp.hpp"

const int PATCH_SIZE = 9;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

DEFINE_string(output_file, "keypoints.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

typedef std::vector<cv::KeyPoint> KeypointList;

// Converts a cv::KeyPoint to a RigidFeature.
RigidFeature keypointToRigidFeature(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return RigidFeature(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

// Renders a keypoint on top of an image with a random color.
void drawKeypoint(cv::Mat& image, const cv::KeyPoint& keypoint) {
  // Generate a random color.
  cv::Scalar color = randomColor(SATURATION, BRIGHTNESS);

  // Convert to our format.
  RigidFeature feature = keypointToRigidFeature(keypoint);

  // Warp is just for drawing. This feels weird.
  RigidWarp warp(PATCH_SIZE);
  warp.draw(image, feature.data(), PATCH_SIZE, color);
}

// Renders all keypoints over an image with random colors.
void drawKeypoints(cv::Mat& image, const KeypointList& keypoints) {
  // Draw each keypoint with a random color.
  std::for_each(keypoints.begin(), keypoints.end(),
      boost::bind(drawKeypoint, boost::ref(image), _1));
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Visualizes the keypoints detected in an image." << std::endl;
  usage << std::endl;
  usage << argv[0] << " image keypoints" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string image_file = argv[1];
  std::string keypoints_file = argv[2];
  std::string output_file = FLAGS_output_file;

  bool ok;

  // Load images.
  cv::Mat image;
  cv::Mat gray_image;
  ok = readImage(image_file, image, gray_image);
  if (!ok) {
    std::cerr << "could not load image" << std::endl;
    return 1;
  }

  // Load keypoints.
  KeypointList keypoints;
  ok = loadKeypoints(keypoints_file, keypoints);
  if (!ok) {
    std::cerr << "could not load keypoints" << std::endl;
    return 1;
  }

  // Visualize matches.
  drawKeypoints(image, keypoints);

  if (FLAGS_save) {
    cv::imwrite(output_file, image);
  }

  if (FLAGS_display) {
    cv::imshow("keypoints", image);
    cv::waitKey(0);
  }

  return 0;
}
