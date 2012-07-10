#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <gflags/gflags.h>
#include "read_image.hpp"
#include "descriptor.hpp"
#include "keypoint.hpp"
#include "match.hpp"
#include "draw_matches.hpp"

DEFINE_string(output_file, "matches.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show matches?");

typedef std::vector<cv::KeyPoint> KeyPointList;

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Visualizes matches between a pair of images." << std::endl;
  usage << std::endl;
  usage << argv[0] << " matches image1 image2 keypoints1 keypoints2" <<
    std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string matches_file = argv[1];
  std::string image_file1 = argv[2];
  std::string image_file2 = argv[3];
  std::string keypoints_file1 = argv[4];
  std::string keypoints_file2 = argv[5];
  std::string output_file = FLAGS_output_file;

  bool ok;

  // Load matches.
  MatchList matches;
  ok = loadMatches(matches_file, matches);
  if (!ok) {
    std::cerr << "could not load matches" << std::endl;
    return 1;
  }

  // Load images.
  cv::Mat image1;
  cv::Mat image2;
  cv::Mat gray;
  ok = readImage(image_file1, image1, gray);
  if (!ok) {
    std::cerr << "could not load image" << std::endl;
    return 1;
  }
  ok = readImage(image_file2, image2, gray);
  if (!ok) {
    std::cerr << "could not load image" << std::endl;
    return 1;
  }

  // Load keypoints.
  KeyPointList keypoints1;
  KeyPointList keypoints2;
  ok = loadKeyPoints(keypoints_file1, keypoints1);
  if (!ok) {
    std::cerr << "could not load keypoints" << std::endl;
    return 1;
  }
  ok = loadKeyPoints(keypoints_file2, keypoints2);
  if (!ok) {
    std::cerr << "could not load keypoints" << std::endl;
    return 1;
  }

  // Visualize matches.
  cv::Mat render;
  drawMatches(keypoints1, keypoints2, matches, image1, image2, render);

  if (FLAGS_save) {
    cv::imwrite(output_file, render);
  }
  if (FLAGS_display) {
    cv::imshow("matches", render);
    cv::waitKey(0);
  }

  return 0;
}
