#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "read_image.hpp"
#include "similarity_warp.hpp"
#include "similarity_feature.hpp"
#include "descriptor.hpp"
#include "sift.hpp"

// for doing fixed point arithmetic
const int SIFT_FIXPT_SCALE = 48;
// assumed gaussian blur for input image
const float SIFT_INIT_SIGMA = 0.5f;

const int MAX_NUM_FEATURES = 100;
const int NUM_OCTAVE_LAYERS = 3;
const double CONTRAST_THRESHOLD = 0.04;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

void printReport(const std::vector<Descriptor>& descriptors1,
                 const std::vector<Descriptor>& descriptors2) {
  typedef std::vector<Descriptor> DescriptorList;

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

  std::cerr << "found " << num_features << " features" << std::endl;
  std::cerr << "average error: " << mean_error << std::endl;
  std::cerr << "max error: " << max_error << std::endl;
  std::cerr << "num exact: " << num_exact << std::endl;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " image" << std::endl;
    return 1;
  }

  std::string image_file = argv[1];

  bool ok = true;

  // Read image.
  cv::Mat integer_image;
  cv::Mat color_image;
  ok = readImage(image_file, color_image, integer_image);
  if (!ok) {
    std::cerr << "could not read image" << std::endl;
    return 1;
  }

  typedef std::vector<Descriptor> DescriptorList;
  typedef std::vector<cv::KeyPoint> KeypointList;
  typedef std::vector<SimilarityFeature> FeatureList;

  KeypointList keypoints;
  FeatureList features;
  DescriptorList cv_descriptors;

  // Get keypoints and descriptors using cv::SIFT.
  // Here we actually want to keep the keypoints to compare against.
  {
    // Extract SIFT keypoints and descriptors.
    cv::Mat descriptor_table;
    cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, CONTRAST_THRESHOLD,
        EDGE_THRESHOLD, SIGMA);
    sift(integer_image, cv::noArray(), keypoints, descriptor_table, false);

    // Convert.
    extractSimilarityFeaturesFromKeypoints(keypoints, features);
    extractDescriptorsFromMatrix(descriptor_table, cv_descriptors);
  }

  SiftExtractor sift(integer_image, NUM_OCTAVE_LAYERS, SIGMA);

  FeatureList::const_iterator feature = features.begin();
  KeypointList::const_iterator cv_keypoint = keypoints.begin();
  DescriptorList::const_iterator cv_descriptor = cv_descriptors.begin();

  std::cout << "x\ty\tsize\tangle\toctave\tlayer\txi\terror" << std::endl;

  while (cv_keypoint != keypoints.end()) {
    cv::KeyPoint my_keypoint = sift.featureToRegisteredKeypoint(*feature);

    // Compute descriptor.
    Descriptor my_descriptor;
    sift.extractDescriptorFromKeypoint(my_keypoint, my_descriptor);

    std::vector<double> diff;
    cv::subtract(my_descriptor.data, cv_descriptor->data, diff);
    double error = cv::norm(diff);

    double norm = cv::norm(cv_descriptor->data);
    error /= norm;

    std::cout << cv_keypoint->pt.x << "\t";
    std::cout << cv_keypoint->pt.y << "\t";
    std::cout << cv_keypoint->size << "\t";
    std::cout << cv_keypoint->angle << "\t";
    std::cout << (cv_keypoint->octave & 255) << "\t";
    std::cout << ((cv_keypoint->octave >> 8) & 255) << "\t";
    std::cout << ((cv_keypoint->octave >> 16) & 255) / 255. - 0.5 << std::endl;

    std::cout << my_keypoint.pt.x << "\t";
    std::cout << my_keypoint.pt.y << "\t";
    std::cout << my_keypoint.size << "\t";
    std::cout << my_keypoint.angle << "\t";
    std::cout << (my_keypoint.octave & 255) << "\t";
    std::cout << ((my_keypoint.octave >> 8) & 255) << "\t";
    std::cout << 0 << "\t";
    std::cout << error << std::endl;

    std::cout << std::endl;

    ++feature;
    ++cv_keypoint;
    ++cv_descriptor;
  }

  // Now try and compute the same features but without OpenCV's extra tags.
  DescriptorList my_descriptors;
  sift.extractDescriptors(features, my_descriptors);

  // Display stats.
  printReport(cv_descriptors, my_descriptors);

  return 0;
}
