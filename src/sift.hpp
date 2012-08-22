#ifndef SIFT_HPP_
#define SIFT_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "similarity_feature.hpp"
#include "descriptor.hpp"

class SiftExtractor {
  public:
    SiftExtractor(const cv::Mat& image, int num_octave_layers, double sigma);

    // Extracts descriptors for a set of features.
    void extractDescriptors(const std::vector<SimilarityFeature>& features,
                            std::vector<Descriptor>& descriptors) const;

    // Extracts a single descriptor. Less efficient.
    void extractDescriptor(const SimilarityFeature& feature,
                           Descriptor& descriptor) const;

    // Extracts a single descriptor. Less efficient.
    void extractDescriptorFromKeypoint(const cv::KeyPoint& keypoint,
                                       Descriptor& descriptor) const;

    // Returns a cv::KeyPoint with necessary information.
    cv::KeyPoint featureToRegisteredKeypoint(
        const SimilarityFeature& feature) const;

    // Finds the octave and layer from which to compute the descriptor.
    void calculatePyramidPosition(double size, int& octave, int& layer) const;

  private:
    std::vector<cv::Mat> pyramid_;
    int num_octave_layers_;
    double sigma_;
};

SimilarityFeature keypointToSimilarityFeature(const cv::KeyPoint& keypoint);

void extractSimilarityFeaturesFromKeypoints(
    const std::vector<cv::KeyPoint>& keypoints,
    std::vector<SimilarityFeature>& features);

void extractDescriptorsFromMatrix(const cv::Mat& matrix,
                                  std::vector<Descriptor>& descriptors);


#endif
