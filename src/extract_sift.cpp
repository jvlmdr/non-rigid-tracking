#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cmath>
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

const int SIFT_FIXPT_SCALE = 48;
// assumed gaussian blur for input image
const float SIFT_INIT_SIGMA = 0.5f;

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

void extractDescriptorFromRow(const cv::Mat& row, Descriptor& descriptor) {
  std::copy(row.begin<float>(), row.end<float>(),
      std::back_inserter(descriptor.data));
}

void extractDescriptorsFromMatrix(const cv::Mat& matrix,
                                  std::vector<Descriptor>& descriptors) {
  // Extract descriptors from rows of matrix.
  for (int i = 0; i < matrix.rows; i += 1) {
    // Create a new descriptor.
    descriptors.push_back(Descriptor());

    extractDescriptorFromRow(matrix.row(i), descriptors.back());
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
  // Extract SIFT keypoints and descriptors.
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptor_table;
  cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, CONTRAST_THRESHOLD,
      EDGE_THRESHOLD, SIGMA);
  sift(image, cv::noArray(), keypoints, descriptor_table, false);

  // Convert.
  extractRigidFeaturesFromKeypoints(keypoints, features);
  extractDescriptorsFromMatrix(descriptor_table, descriptors);
}

void calculatePyramidPosition(double size, int& octave, int& layer) {
  // I think it should be
  //   scale = size / patch_size
  //         = 2 ^ (octave + layer / num_octave_layers)
  // double scale = feature->size / width;
  // But let's follow the same formula as OpenCV.
  //   size = sigma * 2 ^ (octave + layer / num_octave_layers + 1)
  double scale = size / SIGMA / 2.;
  double log2_scale = std::log(scale) / std::log(2.);

  octave = std::floor(log2_scale);
  double remainder = NUM_OCTAVE_LAYERS * (log2_scale - octave);
  // I thought rounding down would be more accurate here,
  // but OpenCV seems to round.
  layer = std::floor(remainder + 0.5);

  // Prefer to use the higher resolution version.
  if (layer == 0 && octave > 0) {
    octave -= 1;
    layer += NUM_OCTAVE_LAYERS;
  }

  // If it's negative then we'll just have to interpolate.
  if (octave < 0) {
    octave = 0;
    layer = 0;
  }
}

// A "registered" keypoint is one that knows its octave and layer.
cv::KeyPoint featureToRegisteredKeypoint(const RigidFeature& feature) {
  // Unused parameters.
  float response = 0;
  int class_id = -1;

  // Carry over.
  double x = feature.x;
  double y = feature.y;
  double size = feature.size;
  double theta = feature.theta * 180. / M_PI;

  // Find position in discretized scale space.
  int octave;
  int layer;
  calculatePyramidPosition(feature.size, octave, layer);

  // Package up into one integer.
  if (layer != (layer & 255)) {
    throw std::runtime_error("layer did not fit in 8 bits");
  }
  if (octave != (octave & 255)) {
    throw std::runtime_error("octave did not fit in 8 bits");
  }
  octave = (layer << 8) | octave;

  // Create keypoint.
  return cv::KeyPoint(x, y, size, theta, response, octave, class_id);
}

void extractSiftDescriptor(const std::vector<cv::Mat>& pyramid,
                           const cv::KeyPoint& keypoint,
                           Descriptor& descriptor) {
  std::vector<cv::KeyPoint> keypoints;
  keypoints.push_back(keypoint);

  // Note: This is not part of the API. Manually exposed by modifying header.
  // Tested with OpenCV 2.4.1 only.
  cv::Mat descriptor_table = cv::Mat_<float>(1, 128);
  cv::calcSiftDescriptors(pyramid, keypoints, descriptor_table,
      NUM_OCTAVE_LAYERS);

  // Convert to our format.
  extractDescriptorFromRow(descriptor_table.row(0), descriptor);
}

void makePyramid(const cv::Mat& byte_image, std::vector<cv::Mat>& pyramid) {
  cv::Mat image;
  byte_image.convertTo(image, CV_16S, SIFT_FIXPT_SCALE, 0);

  // Blur base level.
  double sig_diff_sqr = SIGMA * SIGMA - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA;
  double sig_diff = std::sqrt(std::max(sig_diff_sqr, 0.01));
  cv::GaussianBlur(image, image, cv::Size(), sig_diff, sig_diff);

  // Compute image pyramid.
  int min_dim = std::min(image.cols, image.rows);
  double log2_min_dim = std::log(double(min_dim)) / std::log(2.);
  int num_octaves = std::floor(log2_min_dim + 0.5) - 2;

  cv::SIFT sift(0, NUM_OCTAVE_LAYERS);
  sift.buildGaussianPyramid(image, pyramid, num_octaves);
}

void extractSiftDescriptors(const cv::Mat& byte_image,
                            const std::vector<RigidFeature>& features,
                            std::vector<Descriptor>& descriptors,
                            int width) {
  typedef std::vector<RigidFeature> FeatureList;

  // Convert each feature to a registered keypoint.
  std::vector<cv::KeyPoint> keypoints;
  std::transform(features.begin(), features.end(),
      std::back_inserter(keypoints), featureToRegisteredKeypoint);

  std::vector<cv::Mat> pyramid;
  makePyramid(byte_image, pyramid);

  // Note: This is not part of the API. Manually exposed by modifying header.
  // Tested with OpenCV 2.4.1 only.
  cv::Mat descriptor_table = cv::Mat_<float>(keypoints.size(), 128);
  cv::calcSiftDescriptors(pyramid, keypoints, descriptor_table,
      NUM_OCTAVE_LAYERS);

  // Convert to our format.
  extractDescriptorsFromMatrix(descriptor_table, descriptors);
}

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
  typedef std::vector<RigidFeature> FeatureList;

  KeypointList keypoints;
  FeatureList features;
  DescriptorList cv_descriptors;

  // Get keypoints and descriptors using cv::SIFT.
  {
    // Extract SIFT keypoints and descriptors.
    cv::Mat descriptor_table;
    cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, CONTRAST_THRESHOLD,
        EDGE_THRESHOLD, SIGMA);
    sift(integer_image, cv::noArray(), keypoints, descriptor_table, false);

    // Convert.
    extractRigidFeaturesFromKeypoints(keypoints, features);
    extractDescriptorsFromMatrix(descriptor_table, cv_descriptors);
  }

  // Initialize a scale-space pyramid.
  std::vector<cv::Mat> pyramid;
  makePyramid(integer_image, pyramid);

  FeatureList::const_iterator feature = features.begin();
  KeypointList::const_iterator cv_keypoint = keypoints.begin();
  DescriptorList::const_iterator cv_descriptor = cv_descriptors.begin();

  std::cerr << "x\ty\tsize\tangle\toctave\tlayer\txi\terror" << std::endl;

  while (cv_keypoint != keypoints.end()) {
    cv::KeyPoint my_keypoint = featureToRegisteredKeypoint(*feature);

    // Compute descriptor.
    Descriptor my_descriptor;
    extractSiftDescriptor(pyramid, my_keypoint, my_descriptor);

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
  extractSiftDescriptors(integer_image, features, my_descriptors, PATCH_SIZE);

  // Display stats.
  printReport(cv_descriptors, my_descriptors);

  return 0;
}
