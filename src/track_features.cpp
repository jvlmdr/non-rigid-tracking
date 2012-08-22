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
#include "track.hpp"
#include "tracker.hpp"
#include "random_color.hpp"
#include "warp.hpp"
#include "similarity_warp.hpp"
#include "similarity_feature.hpp"
#include "flow.hpp"

// Size of window to track.
const int PATCH_SIZE = 9;
const int NUM_PIXELS = PATCH_SIZE * PATCH_SIZE;

// Lucas-Kanade optimization settings.
const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-4;
const double GRADIENT_TOLERANCE = 0;
const double PARAMETER_TOLERANCE = 1e-4;
const bool ITERATION_LIMIT_IS_FATAL = false;
const double MAX_CONDITION = 1000;

// Do not want to sample below one pixel.
const double MIN_SCALE = 0.5;

// Maximum average intensity difference as a fraction of the range.
// (A value of 1 means anything is permitted.)
const double MAX_AVERAGE_RESIDUAL = 0.1;

const int MAX_NUM_FEATURES = 100;
const int NUM_OCTAVE_LAYERS = 3;
const double CONTRAST_THRESHOLD = 0.04;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

SimilarityFeature keypointToSimilarityFeature(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return SimilarityFeature(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

struct Feature {
  SimilarityFeature state;
  cv::Mat appearance;
  cv::Scalar color;

  Feature() : state(), appearance(), color() {}

  Feature(const SimilarityFeature& state, const cv::Scalar& color)
      : state(state), appearance(), color(color) {}
};

Feature makeRandomColorFeature(const cv::KeyPoint& keypoint) {
  return Feature(keypointToSimilarityFeature(keypoint),
      randomColor(SATURATION, BRIGHTNESS));
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " image-format output-format" <<
      std::endl;
    return 1;
  }

  std::string image_format = argv[1];
  std::string output_format = argv[2];

  int t = 0;
  bool ok = true;

  SimilarityWarp similarity_warp(PATCH_SIZE);
  Warp& warp = similarity_warp;

  WarpTracker tracker(warp, PATCH_SIZE, MAX_NUM_ITERATIONS, FUNCTION_TOLERANCE,
      GRADIENT_TOLERANCE, PARAMETER_TOLERANCE, ITERATION_LIMIT_IS_FATAL,
      MAX_CONDITION);

  typedef std::list<Feature> FeatureList;
  FeatureList features;

  while (ok) {
    // Read image.
    cv::Mat integer_image;
    cv::Mat color_image;
    ok = readImage(makeFilename(image_format, t), color_image, integer_image);
    if (!ok) {
      std::cerr << "could not read image" << std::endl;
      return 1;
    }

    // Convert to floating point.
    cv::Mat image;
    integer_image.convertTo(image, cv::DataType<double>::type, 1. / 255.);

    tracker.feedImage(image);

    if (t == 0) {
      // First frame: get some features.
      std::vector<cv::KeyPoint> keypoints;
      cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, CONTRAST_THRESHOLD,
          EDGE_THRESHOLD, SIGMA);
      sift(integer_image, cv::noArray(), keypoints, cv::noArray(), false);

      // Convert keypoints to our features.
      std::transform(keypoints.begin(), keypoints.end(),
          std::back_inserter(features), makeRandomColorFeature);

      // Sample appearance of each feature.
      for (FeatureList::iterator feature = features.begin();
           feature != features.end();
           ++feature) {
        // Sample patch at new position in image.
        cv::Mat M = warp.matrix(feature->state.data());
        sampleAffinePatch(image, feature->appearance, M, PATCH_SIZE, false);
      }

      std::cerr << "detected " << features.size() << " features" << std::endl;
    } else {
      // Iterate through features and either update or remove each.
      FeatureList::iterator feature = features.begin();

      while (feature != features.end()) {
        bool remove;
        bool tracked = tracker.track(feature->state.data());

        if (!tracked) {
          // Remove from list.
          remove = true;
          std::cerr << "could not track" << std::endl;
        } else {
          // Check that feature did not become too small.
          double scale = feature->state.size / double(PATCH_SIZE);

          if (scale < MIN_SCALE) {
            remove = true;
            std::cerr << "scale too small" << std::endl;
          } else {
            // Sample patch at new position in image.
            cv::Mat M = warp.matrix(feature->state.data());
            cv::Mat appearance;
            sampleAffinePatch(image, appearance, M, PATCH_SIZE, false);

            // Check that the feature looks similar.
            double residual = averageResidual(feature->appearance, appearance);

            if (residual > MAX_AVERAGE_RESIDUAL) {
              remove = true;
              std::cerr << "residual too large" << std::endl;
            } else {
              // Keep the feature!
              remove = false;
              // Swap the image contents.
              feature->appearance = appearance;
            }
          }
        }

        if (remove) {
          feature = features.erase(feature);
        } else {
          ++feature;
        }
      }

      std::cout << "tracked " << features.size() << " features" << std::endl;
    }

    // Visualize positions.
    for (FeatureList::const_iterator feature = features.begin();
         feature != features.end();
         ++feature) {
      warp.draw(color_image, feature->state.data(), PATCH_SIZE, feature->color);
    }

    std::string output_filename = makeFilename(output_format, t);
    cv::imwrite(output_filename, color_image);

    cv::imshow("frame", color_image);
    cv::waitKey(10);

    t += 1;
  }

  return 0;
}
