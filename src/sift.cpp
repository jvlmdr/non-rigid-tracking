#include "sift.hpp"
#include <boost/bind.hpp>
#include <stdexcept>

const int SIFT_FIXPT_SCALE = 48;
const double SIFT_INIT_SIGMA = 0.5;

RigidFeature keypointToRigidFeature(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return RigidFeature(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

////////////////////////////////////////////////////////////////////////////////

void extractRigidFeaturesFromKeypoints(
    const std::vector<cv::KeyPoint>& keypoints,
    std::vector<RigidFeature>& features) {
  // Transform each element.
  std::transform(keypoints.begin(), keypoints.end(),
      std::back_inserter(features), keypointToRigidFeature);
}

void extractDescriptorFromRow(const cv::Mat& row, Descriptor& descriptor) {
  if (row.type() != cv::DataType<float>::type) {
    throw std::runtime_error("expected 32-bit float");
  }

  if (row.total() != 128) {
    throw std::runtime_error("expected 128-dimensional descriptor");
  }

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

void makePyramid(const cv::Mat& byte_image,
                 std::vector<cv::Mat>& pyramid,
                 int num_octave_layers,
                 double sigma) {
  cv::Mat image;
  byte_image.convertTo(image, CV_16S, SIFT_FIXPT_SCALE, 0);

  // Blur base level.
  double sig_diff_sqr = sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA;
  double sig_diff = std::sqrt(std::max(sig_diff_sqr, 0.01));
  cv::GaussianBlur(image, image, cv::Size(), sig_diff, sig_diff);

  // Compute image pyramid.
  int min_dim = std::min(image.cols, image.rows);
  double log2_min_dim = std::log(double(min_dim)) / std::log(2.);
  int num_octaves = std::floor(log2_min_dim + 0.5) - 2;

  cv::SIFT sift(0, num_octave_layers);
  sift.buildGaussianPyramid(image, pyramid, num_octaves);
}

////////////////////////////////////////////////////////////////////////////////

SiftExtractor::SiftExtractor(const cv::Mat& image,
                             int num_octave_layers,
                             double sigma)
    : pyramid_(),
      num_octave_layers_(num_octave_layers),
      sigma_(sigma) {
  // Construct Gaussian pyramid.
  makePyramid(image, pyramid_, num_octave_layers_, sigma_);
}

void SiftExtractor::extractDescriptors(
    const std::vector<RigidFeature>& features,
    std::vector<Descriptor>& descriptors) const {
  // Convert each feature to a registered keypoint.
  std::vector<cv::KeyPoint> keypoints;
  std::transform(features.begin(), features.end(),
      std::back_inserter(keypoints),
      boost::bind(&SiftExtractor::featureToRegisteredKeypoint, *this, _1));

  // Note: This is not part of the API. Manually exposed by modifying OpenCV.
  // Tested with OpenCV 2.4.1 only.
  cv::Mat descriptor_table = cv::Mat_<float>(keypoints.size(), 128);
  cv::calcSiftDescriptors(pyramid_, keypoints, descriptor_table,
      num_octave_layers_);

  // Convert to our format.
  extractDescriptorsFromMatrix(descriptor_table, descriptors);
}

void SiftExtractor::extractDescriptor(const RigidFeature& feature,
                                      Descriptor& descriptor) const {
  // Register the feature in the pyramid.
  cv::KeyPoint keypoint = featureToRegisteredKeypoint(feature);
  extractDescriptorFromKeypoint(keypoint, descriptor);
}

void SiftExtractor::extractDescriptorFromKeypoint(const cv::KeyPoint& keypoint,
                                                  Descriptor& descriptor) const {
  // Add it to a single-element list.
  std::vector<cv::KeyPoint> keypoints;
  keypoints.push_back(keypoint);

  // Note: This is not part of the API. Manually exposed by modifying OpenCV.
  // Tested with OpenCV 2.4.1 only.
  cv::Mat descriptor_table = cv::Mat_<float>(1, 128);
  cv::calcSiftDescriptors(pyramid_, keypoints, descriptor_table,
      num_octave_layers_);

  // Convert to our format.
  extractDescriptorFromRow(descriptor_table.row(0), descriptor);
}

void SiftExtractor::calculatePyramidPosition(double size,
                                             int& octave,
                                             int& layer) const {
  // I think it should be
  //   scale = size / patch_size
  //         = 2 ^ (octave + layer / num_octave_layers)
  // double scale = feature->size / width;
  // But let's follow the same formula as OpenCV.
  //   size = sigma * 2 ^ (octave + layer / num_octave_layers + 1)
  double scale = size / sigma_ / 2.;
  double log2_scale = std::log(scale) / std::log(2.);

  octave = std::floor(log2_scale);
  double remainder = num_octave_layers_ * (log2_scale - octave);
  // I thought rounding down would be more accurate here,
  // but OpenCV seems to round.
  layer = std::floor(remainder + 0.5);

  // Prefer to use the higher resolution version, matching OpenCV.
  if (layer == 0 && octave > 0) {
    octave -= 1;
    layer += num_octave_layers_;
  }

  // If it's negative then we'll just have to interpolate.
  if (octave < 0) {
    octave = 0;
    layer = 0;
  }

  // It should never be above the limit.
  if (octave >= pyramid_.size()) {
    throw std::runtime_error("octave too large!");
  }
}

// A "registered" keypoint is one that knows its octave and layer.
cv::KeyPoint SiftExtractor::featureToRegisteredKeypoint(
    const RigidFeature& feature) const {
  // Unused parameters.
  float response = 0;
  int class_id = -1;

  double x = feature.x;
  double y = feature.y;
  double size = feature.size;
  // Angle must be between 0 and 360!
  double theta = std::fmod(feature.theta, 2 * M_PI) * 180. / M_PI;

  // Find position in discretized scale space.
  int octave;
  int layer;
  calculatePyramidPosition(size, octave, layer);

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


////////////////////////////////////////////////////////////////////////////////

