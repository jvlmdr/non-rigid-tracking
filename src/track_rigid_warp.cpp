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
#include "rigid_warp.hpp"
#include "rigid_feature.hpp"
#include "flow.hpp"

// Size of window to track.
const int PATCH_SIZE = 9;
const int NUM_PIXELS = PATCH_SIZE * PATCH_SIZE;

// Lucas-Kanade optimization settings.
const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-4;
const double GRADIENT_TOLERANCE = 0;
const double PARAMETER_TOLERANCE = 1e-4;
const bool ITERATION_LIMIT_IS_FATAL = true;
const double MAX_CONDITION = 1000;

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

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

////////////////////////////////////////////////////////////////////////////////

RigidFeature keypointToRigidFeature(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return RigidFeature(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

struct ColoredFeature {
  RigidFeature value;
  cv::Scalar color;

  ColoredFeature() : value(), color() {}

  ColoredFeature(const RigidFeature& value, const cv::Scalar& color)
      : value(value), color(color) {}
};

ColoredFeature makeRandomlyColoredFeature(const cv::KeyPoint& keypoint) {
  return ColoredFeature(keypointToRigidFeature(keypoint),
      randomColor(SATURATION, BRIGHTNESS));
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " image-format output-format" <<
      std::endl;
    return 1;
  }

  std::string image_format = argv[1];
  std::string output_format = argv[2];

  int t = 0;
  bool ok = true;
  cv::Mat previous_image;

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = MAX_NUM_ITERATIONS;
  options.function_tolerance = FUNCTION_TOLERANCE;
  options.gradient_tolerance = GRADIENT_TOLERANCE;
  options.parameter_tolerance = PARAMETER_TOLERANCE;

  RigidWarp rigid_warp(PATCH_SIZE);
  Warp& warp = rigid_warp;

  // Start at image center with unit scale and zero orientation.
  typedef std::list<ColoredFeature> FeatureList;
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

    if (t == 0) {
      // Get some features to track.
      std::vector<cv::KeyPoint> keypoints;
      cv::SIFT sift(MAX_NUM_FEATURES, NUM_OCTAVE_LAYERS, CONTRAST_THRESHOLD,
          EDGE_THRESHOLD, SIGMA);
      sift(integer_image, cv::noArray(), keypoints, cv::noArray(), false);

      // Convert keypoints to our features.
      std::transform(keypoints.begin(), keypoints.end(),
          std::back_inserter(features), makeRandomlyColoredFeature);

      std::cerr << "detected " << features.size() << " features" << std::endl;
    } else {
      // Take x and y derivatives.
      cv::Mat ddx_image;
      cv::Mat ddy_image;
      cv::Mat diff = (cv::Mat_<double>(1, 3) << -0.5, 0, 0.5);
      cv::Mat identity = (cv::Mat_<double>(1, 1) << 1);
      cv::sepFilter2D(image, ddx_image, -1, diff, identity);
      cv::sepFilter2D(image, ddy_image, -1, identity, diff);

      FeatureList prev_features;
      prev_features.swap(features);

      std::map<int, ColoredFeature> results;
      int i = 0;

      for (FeatureList::const_iterator prev_feature = prev_features.begin();
           prev_feature != prev_features.end();
           ++prev_feature) {
        // Sample patch from previous image.
        cv::Mat M = warp.matrix(prev_feature->value.data());
        cv::Mat reference;
        sampleAffinePatch(previous_image, reference, M, PATCH_SIZE);

        // Use previous state as initialization (and keep same color).
        ColoredFeature feature = *prev_feature;
        bool tracked = solveFlow(warp, PATCH_SIZE, reference, image, ddx_image,
            ddy_image, feature.value.data(), options, ITERATION_LIMIT_IS_FATAL,
            MAX_CONDITION);

        if (tracked) {
          double scale = feature.value.size / PATCH_SIZE;

          if (scale >= MIN_SCALE) {
            // Check appearance.
            double residual = meanPixelDifference(warp, PATCH_SIZE, reference,
                image, feature.value.data());

            if (residual <= MAX_RESIDUAL) {
              // Retain feature.
              features.push_back(feature);
            } else {
              std::cerr << "not similar enough" << std::endl;
            }
          } else {
            std::cerr << "scale too small" << std::endl;
          }
        }

        i += 1;
      }

      std::cout << "tracked " << features.size() << " features" << std::endl;
    }

    // Visualize.
    for (FeatureList::const_iterator feature = features.begin();
         feature != features.end();
         ++feature) {
      warp.draw(color_image, feature->value.data(), PATCH_SIZE, feature->color);
    }

    std::string output_filename = makeFilename(output_format, t);
    cv::imwrite(output_filename, color_image);

    cv::imshow("frame", color_image);
    cv::waitKey(10);

    image.copyTo(previous_image);
    t += 1;
  }

  return 0;
}
