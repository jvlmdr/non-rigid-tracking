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
#include "sift_feature.hpp"

#include "sift_feature_writer.hpp"
#include "vector_writer.hpp"

const int MAX_NUM_FEATURES = 0;
const int NUM_OCTAVE_LAYERS = 3;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

DEFINE_double(contrast_threshold, 0.04,
    "Constrast threshold for feature detection");

typedef std::vector<SiftFeature> FeatureList;

SiftPosition keypointToSiftPosition(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return SiftPosition(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

void extractFeatures(const cv::Mat& image,
                     FeatureList& features,
                     double threshold) {
  // Clear list.
  features.clear();

  // SIFT settings.
  cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, threshold, EDGE_THRESHOLD,
      SIGMA);

  // Extract SIFT keypoints and descriptors.
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift(image, cv::noArray(), keypoints, descriptors, false);

  // Ensure that descriptors are 32-bit floats.
  CHECK(descriptors.type() == cv::DataType<float>::type);

  int num_features = keypoints.size();

  // Convert to features.
  for (int i = 0; i < num_features; i += 1) {
    SiftFeature feature;
    feature.position = keypointToSiftPosition(keypoints[i]);

    // Copy descriptor contents.
    cv::Mat row = descriptors.row(i);
    feature.descriptor.data.clear();
    std::copy(row.begin<float>(), row.end<float>(),
        std::back_inserter(feature.descriptor.data));

    // Add to list.
    features.push_back(feature);
  }
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Finds SIFT keypoints in an image and extracts descriptors." <<
    std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " image features" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_filename = argv[1];
  std::string features_filename = argv[2];
  double CONTRAST_THRESHOLD = FLAGS_contrast_threshold;

  // Read image.
  cv::Mat color_image;
  cv::Mat image;
  bool ok = readImage(image_filename, color_image, image);
  CHECK(ok) << "Could not read image";

  FeatureList features;
  extractFeatures(image, features, CONTRAST_THRESHOLD);

  LOG(INFO) << "Found " << features.size() << " features";

  // Save out to file.
  SiftFeatureWriter feature_writer;
  ok = saveList(features_filename, features, feature_writer);
  CHECK(ok) << "Could not save descriptors";

  return 0;
}
