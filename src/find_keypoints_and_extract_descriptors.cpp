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
#include "rigid_feature.hpp"
#include "sift.hpp"
#include "writer.hpp"
#include "vector_writer.hpp"

const int MAX_NUM_FEATURES = 0;
const int NUM_OCTAVE_LAYERS = 3;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

DEFINE_double(contrast_threshold, 0.04,
    "Constrast threshold for feature detection");

std::string usage(const char* argv0) {
  std::ostringstream usage;

  usage << "Finds SIFT keypoints in an image and extracts descriptors." <<
    std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv0 << " image features" << std::endl;

  return usage.str();
}

struct Feature {
  // This is "position" in a general sense. More like 2D pose.
  RigidFeature position;
  // Fixed-size representation of appearance.
  Descriptor descriptor;
};

class FeatureWriter : public Writer<Feature> {
  public:
    ~FeatureWriter() {}

    void write(cv::FileStorage& file, const Feature& feature) {
      file << "x" << feature.position.x;
      file << "y" << feature.position.y;
      file << "size" << feature.position.size;
      file << "angle" << feature.position.theta;
      file << "descriptor";
      feature.descriptor.write(file);
    }
};

typedef std::vector<Feature> FeatureList;

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
    Feature feature;
    feature.position = keypointToRigidFeature(keypoints[i]);

    // Copy descriptor contents.
    cv::Mat row = descriptors.row(i);
    feature.descriptor.data.clear();
    std::copy(row.begin<float>(), row.end<float>(),
        std::back_inserter(feature.descriptor.data));

    // Add to list.
    features.push_back(feature);
  }
}

int main(int argc, char** argv) {
  google::SetUsageMessage(usage(argv[0]));
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string image_filename = argv[1];
  std::string features_filename = argv[2];
  double CONTRAST_THRESHOLD = FLAGS_contrast_threshold;

  // Read image.
  cv::Mat color_image;
  cv::Mat image;
  bool ok = readImage(image_filename, color_image, image);
  CHECK(ok) << "Could not read image";

  LOG(INFO) << "Extracting features...";

  FeatureList features;
  extractFeatures(image, features, CONTRAST_THRESHOLD);

  LOG(INFO) << "Found " << features.size() << " features";

  // Save out to file.
  FeatureWriter feature_writer;
  VectorWriter<Feature> writer(feature_writer);
  ok = save(features_filename, features, writer);

  CHECK(ok) << "Could not save descriptors";

  return 0;
}
