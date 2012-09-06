#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include "read_image.hpp"
#include "detect_sift.hpp"
#include "extract_sift.hpp"

typedef std::vector<SiftPosition> PositionList;
typedef std::vector<Descriptor> DescriptorList;
typedef std::vector<SiftFeature> FeatureList;

const int MAX_NUM_FEATURES = 100;
const int NUM_OCTAVE_LAYERS = 3;
const double CONTRAST_THRESHOLD = 0.04;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

void extractPositionsAndDescriptors(const FeatureList& features,
                                    PositionList& positions,
                                    DescriptorList& descriptors) {
  positions.clear();
  descriptors.clear();

  FeatureList::const_iterator feature;
  for (feature = features.begin(); feature != features.end(); ++feature) {
    positions.push_back(feature->position);
    descriptors.push_back(feature->descriptor);
  }
}

void printReport(const DescriptorList& descriptors1,
                 const DescriptorList& descriptors2) {
  // Compare the descriptors.
  int num_features = descriptors1.size();

  DescriptorList::const_iterator descriptor1 = descriptors1.begin();
  DescriptorList::const_iterator descriptor2 = descriptors2.begin();

  double mean_error = 0;
  double max_error = 0;
  int num_exact = 0;

  while (descriptor1 != descriptors1.end()) {
    double norm1 = cv::norm(descriptor1->data);
    double norm2 = cv::norm(descriptor2->data);

    std::vector<double> diff;
    cv::subtract(descriptor1->data, descriptor2->data, diff);
    double error = cv::norm(diff);

    if (error == 0) {
      num_exact += 1;
    }

    error = error / std::sqrt(norm1 * norm2);
    mean_error += error;
    max_error = std::max(max_error, error);

    ++descriptor1;
    ++descriptor2;
  }

  mean_error /= num_features;

  LOG(INFO) << "Found " << num_features << " features";
  LOG(INFO) << "Average error: " << mean_error;
  LOG(INFO) << "Max error: " << max_error;
  LOG(INFO) << "Num exact: " << num_exact;
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Tests arbitrary SIFT descriptor extraction." << std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " image" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 2) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " image" << std::endl;
    return 1;
  }

  std::string image_file = argv[1];

  bool ok = true;

  // Read image.
  cv::Mat image;
  cv::Mat color_image;
  ok = readImage(image_file, color_image, image);
  CHECK(ok) << "Could not load image";

  SiftOptions options;
  options.max_num_features = MAX_NUM_FEATURES;
  options.num_octave_layers = NUM_OCTAVE_LAYERS;
  options.contrast_threshold = CONTRAST_THRESHOLD;
  options.edge_threshold = EDGE_THRESHOLD;
  options.sigma = SIGMA;

  // Get keypoints and descriptors using cv::SIFT.
  FeatureList features;
  extractFeatures(image, features, options);

  // Extract the keypoints and descriptors.
  PositionList positions;
  DescriptorList opencv_descriptors;
  extractPositionsAndDescriptors(features, positions, opencv_descriptors);

  // Re-extract descriptors without metadata tags.
  SiftExtractor extractor(image, NUM_OCTAVE_LAYERS, SIGMA);
  DescriptorList my_descriptors;
  extractor.extractDescriptors(positions, my_descriptors);

  // Display stats.
  printReport(opencv_descriptors, my_descriptors);

  return 0;
}
