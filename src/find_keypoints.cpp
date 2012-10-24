#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include "read_image.hpp"
#include "detect_sift.hpp"

#include "sift_feature_writer.hpp"
#include "iterator_writer.hpp"

const int MAX_NUM_FEATURES = 0;
const int NUM_OCTAVE_LAYERS = 3;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

DEFINE_double(contrast_threshold, 0.04,
    "Constrast threshold for feature detection");

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

  // Read image.
  cv::Mat color_image;
  cv::Mat image;
  bool ok = readImage(image_filename, color_image, image);
  CHECK(ok) << "Could not read image";

  SiftOptions options;
  options.max_num_features = MAX_NUM_FEATURES;
  options.num_octave_layers = NUM_OCTAVE_LAYERS;
  options.contrast_threshold = FLAGS_contrast_threshold;
  options.edge_threshold = EDGE_THRESHOLD;
  options.sigma = SIGMA;

  std::vector<SiftFeature> features;
  extractFeatures(image, features, options);

  LOG(INFO) << "Found " << features.size() << " features";

  // Save out to file.
  SiftFeatureWriter feature_writer;
  ok = saveList(features_filename, features, feature_writer);
  CHECK(ok) << "Could not save descriptors";

  return 0;
}
