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
#include <glog/logging.h>
#include "read_image.hpp"
#include "descriptor.hpp"
#include "match.hpp"
#include "random_color.hpp"
#include "similarity_feature.hpp"
#include "draw_similarity_feature.hpp"
#include "similarity_feature_reader.hpp"
#include "vector_reader.hpp"

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const int LINE_THICKNESS = 1;

DEFINE_string(output_file, "keypoints.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

typedef std::vector<SimilarityFeature> FeatureList;

// Converts a cv::KeyPoint to a SimilarityFeature.
SimilarityFeature keypointToSimilarityFeature(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return SimilarityFeature(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

// Renders a keypoint on top of an image with a random color.
void drawFeature(cv::Mat& image, const SimilarityFeature& feature) {
  // Generate a random color.
  cv::Scalar color = randomColor(SATURATION, BRIGHTNESS);
  drawSimilarityFeature(image, feature, color, LINE_THICKNESS);
}

// Renders all keypoints over an image with random colors.
void drawSimilarityFeatures(cv::Mat& image, const FeatureList& features) {
  // Draw each keypoint with a random color.
  std::for_each(features.begin(), features.end(),
      boost::bind(drawFeature, boost::ref(image), _1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Visualizes the keypoints detected in an image." << std::endl;
  usage << std::endl;
  usage << argv[0] << " image keypoints" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
}

int main(int argc, char** argv) {
  init(argc, argv);

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
  CHECK(ok) << "Could not load image";

  // Load keypoints.
  FeatureList features;
  SimilarityFeatureReader feature_reader;
  ok = loadList(keypoints_file, features, feature_reader);
  CHECK(ok) << "Could not load keypoints";
  LOG(INFO) << "Loaded " << features.size() << " keypoints" << std::endl;

  // Visualize matches.
  drawSimilarityFeatures(image, features);

  if (FLAGS_save) {
    cv::imwrite(output_file, image);
  }

  if (FLAGS_display) {
    cv::imshow("keypoints", image);
    cv::waitKey(0);
  }

  return 0;
}
