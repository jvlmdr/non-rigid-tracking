#ifndef EXTRACT_SIFT_HPP_
#define EXTRACT_SIFT_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "sift_position.hpp"
#include "descriptor.hpp"

// Extracts SIFT descriptors at arbitrary detections.
class SiftExtractor {
  public:
    SiftExtractor(const cv::Mat& image, int num_octave_layers, double sigma);

    // Extracts descriptors for a set of features.
    void extractDescriptors(const std::vector<SiftPosition>& features,
                            std::vector<Descriptor>& descriptors) const;

    // Extracts a single descriptor. Less efficient.
    void extractDescriptor(const SiftPosition& feature,
                           Descriptor& descriptor) const;

    // Extracts a single descriptor. Less efficient.
    void extractDescriptorFromKeypoint(const cv::KeyPoint& keypoint,
                                       Descriptor& descriptor) const;

    // Returns a cv::KeyPoint with necessary information.
    cv::KeyPoint featureToRegisteredKeypoint(
        const SiftPosition& feature) const;

    // Finds the octave and layer from which to compute the descriptor.
    void calculatePyramidPosition(double size, int& octave, int& layer) const;

  private:
    std::vector<cv::Mat> pyramid_;
    int num_octave_layers_;
    double sigma_;
};

#endif
