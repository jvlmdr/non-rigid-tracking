#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <ceres/ceres.h>
#include "read_image.hpp"
#include "warp.hpp"
#include "rigid_warp.hpp"
#include "rigid_feature.hpp"
#include "descriptor.hpp"

// Size of window to track.
const int PATCH_SIZE = 9;
const int NUM_PIXELS = PATCH_SIZE * PATCH_SIZE;

// Lucas-Kanade optimization settings.
const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-4;
const double GRADIENT_TOLERANCE = 0;
const double PARAMETER_TOLERANCE = 1e-4;
const bool ITERATION_LIMIT_IS_FATAL = true;
const double MAX_CONDITION = 100;

// Do not want to sample below one pixel.
const double MIN_SCALE = 0.5;

// Maximum average intensity difference as a fraction of the range.
// (A value of 1 means anything is permitted.)
const double MAX_RESIDUAL = 0.1;

const int MAX_NUM_FEATURES = 100;
const int NUM_OCTAVE_LAYERS = 3;
const double CONTRAST_THRESHOLD = 0.04;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

RigidFeature keypointToRigidFeature(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return RigidFeature(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

void extractRigidFeaturesFromKeypoints(
    const std::vector<cv::KeyPoint>& keypoints,
    std::vector<RigidFeature>& features) {
  // Transform each element.
  std::transform(keypoints.begin(), keypoints.end(),
      std::back_inserter(features), keypointToRigidFeature);
}

void extractDescriptorsFromMatrix(const cv::Mat& matrix,
                                  std::vector<Descriptor>& descriptors) {
  // Extract descriptors from rows of matrix.
  for (int i = 0; i < matrix.rows; i += 1) {
    // Add a descriptor to the vector.
    descriptors.push_back(Descriptor());

    // Copy row into descriptor.
    cv::Mat row = matrix.row(i);
    std::copy(row.begin<float>(), row.end<float>(),
        std::back_inserter(descriptors.back().data));
  }
}

void findFeatures(const cv::Mat& image, std::vector<RigidFeature>& features) {
  // Extract SIFT keypoints.
  std::vector<cv::KeyPoint> keypoints;
  cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, CONTRAST_THRESHOLD,
      EDGE_THRESHOLD, SIGMA);
  sift(image, cv::noArray(), keypoints, cv::noArray(), false);

  // Convert.
  extractRigidFeaturesFromKeypoints(keypoints, features);
}

void findFeaturesAndExtractDescriptors(const cv::Mat& image,
                                       std::vector<RigidFeature>& features,
                                       std::vector<Descriptor>& descriptors) {
  // Extract SIFT keypointsa and descriptors.
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptor_table;
  cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, CONTRAST_THRESHOLD,
      EDGE_THRESHOLD, SIGMA);
  sift(image, cv::noArray(), keypoints, descriptor_table, false);

  // Convert.
  extractRigidFeaturesFromKeypoints(keypoints, features);
  extractDescriptorsFromMatrix(descriptor_table, descriptors);
}

void extractSiftDescriptors(const cv::Mat& image,
                            const std::vector<RigidFeature>& features,
                            std::vector<Descriptor> descriptors) {
  typedef std::vector<RigidFeature> FeatureList;
  typedef std::vector<Descriptor> DescriptorList;

  for (FeatureList::const_iterator feature = features.begin();
       feature != features.end();
       ++feature) {
    std::cout << feature->size << std::endl;
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " image descriptors" <<
      std::endl;
    return 1;
  }

  std::string image_file = argv[1];
  std::string output_file = argv[2];

  bool ok = true;

  // Read image.
  cv::Mat integer_image;
  cv::Mat color_image;
  ok = readImage(image_file, color_image, integer_image);
  if (!ok) {
    std::cerr << "could not read image" << std::endl;
    return 1;
  }

  // Extract SIFT features.
  std::vector<RigidFeature> features;
  std::vector<Descriptor> descriptors1;
  findFeaturesAndExtractDescriptors(integer_image, features, descriptors1);

  // Now try and compute the same features but without OpenCV's extra tags.
  std::vector<Descriptor> descriptors2;
  extractSiftDescriptors(integer_image, features, descriptors2);

  std::cerr << "detected " << features.size() << " features" << std::endl;

  return 0;
}
