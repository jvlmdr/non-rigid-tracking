#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <gflags/gflags.h>
#include <boost/format.hpp>
#include <boost/random.hpp>
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
const int PATCH_SIZE = 32;
const int NUM_PIXELS = PATCH_SIZE * PATCH_SIZE;

const double INITIAL_SIZE = 256;
const double SIGMA_X = 0.01 * INITIAL_SIZE;
const double SIGMA_Y = 0.01 * INITIAL_SIZE;
const double SIGMA_SIZE = 0.01 * INITIAL_SIZE;
const double SIGMA_THETA = 1. * M_PI / 180.;

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

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Tests rigid warp tracking." << std::endl;
  usage << std::endl;
  usage << argv[0] << " image" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 2) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string image_file = argv[1];
  bool ok;

  // Load image.
  cv::Mat integer_image;
  cv::Mat color_image;
  ok = readImage(image_file, color_image, integer_image);
  if (!ok) {
    std::cerr << "could not read image" << std::endl;
    return 1;
  }

  // Convert to floating point.
  cv::Mat image;
  integer_image.convertTo(image, cv::DataType<double>::type, 1. / 255.);

  // Create a warp for sampling patches at the correct resolution.
  SimilarityWarp patch_warp(PATCH_SIZE);
  // Create a warp for applying the same transform to the input image.
  SimilarityWarp image_warp(INITIAL_SIZE);

  // Create tracker using specified warp.
  WarpTracker tracker(patch_warp, PATCH_SIZE, MAX_NUM_ITERATIONS,
      FUNCTION_TOLERANCE, GRADIENT_TOLERANCE, PARAMETER_TOLERANCE,
      ITERATION_LIMIT_IS_FATAL, MAX_CONDITION);

  // Give the first image to the tracker.
  tracker.feedImage(image);

  int width = image.cols;
  int height = image.rows;

  SimilarityFeature feature((width - 1) / 2., (height - 1) / 2., INITIAL_SIZE, 0);
  cv::Scalar ground_truth_color(0x00, 0xCC, 0x00);
  cv::Scalar test_color(0x00, 0x00, 0xFF);

  boost::random::mt19937 gen;
  boost::random::normal_distribution<> dist(0, 1);

  for (int n = 0; n < 1024; n += 1) {
    // Sample the previous image at the previous warp.
    cv::Mat M;
    M = patch_warp.matrix(feature.data());
    cv::Mat patch;
    sampleAffinePatch(image, patch, M, PATCH_SIZE, false);

    // Perturb the image.
    double delta_x = dist(gen) * SIGMA_X;
    double delta_y = dist(gen) * SIGMA_Y;
    double delta_size = dist(gen) * SIGMA_SIZE;
    double delta_theta = dist(gen) * SIGMA_THETA;

    SimilarityFeature perturbed(feature);
    perturbed.x += delta_x;
    perturbed.y += delta_y;
    perturbed.size += delta_size;
    perturbed.theta += delta_theta;

    // Warp the original image according to the new warp.
    M = image_warp.matrix(perturbed.data());
    cv::Mat next_image;
    sampleAffine(image, next_image, M, image.size(), true);

    // Give new image to the tracker.
    tracker.feedImage(next_image);

    // Solve for the transform.
    bool failed;
    bool tracked = tracker.track(feature.data());

    if (!tracked) {
      // Remove from list.
      failed = true;
      std::cerr << "could not track" << std::endl;
    } else {
      // Check that feature did not become too small.
      double scale = feature.size / double(PATCH_SIZE);

      if (scale < MIN_SCALE) {
        failed = true;
        std::cerr << "scale too small" << std::endl;
      } else {
        // Sample patch at new position in image.
        M = patch_warp.matrix(feature.data());
        cv::Mat next_patch;
        sampleAffinePatch(image, next_patch, M, PATCH_SIZE, false);

        // Check that the feature looks similar.
        double residual = averageResidual(patch, next_patch);

        if (residual > MAX_AVERAGE_RESIDUAL) {
          failed = true;
          std::cerr << "residual too large" << std::endl;
        } else {
          // Keep the feature!
          failed = false;
        }
      }
    }

    if (failed) {
      std::cerr << "could not track" << std::endl;
      return 1;
    }

    // Convert gray image back to color.
    next_image.convertTo(integer_image, cv::DataType<uint8_t>::type, 255.);
    cv::Mat render;
    cvtColor(integer_image, render, CV_GRAY2BGR);

    // Draw ground truth feature on image.
    patch_warp.draw(render, perturbed.data(), PATCH_SIZE, ground_truth_color);
    patch_warp.draw(render, feature.data(), PATCH_SIZE, test_color);
    cv::imshow("ground truth", render);
    cv::waitKey(10);

    feature = perturbed;
  }

  /*
  // Sample patch at original position in image.
  cv::Mat prev_appearance;
  {
    cv::Mat M = warp.matrix(feature.data());
    sampleAffinePatch(image, prev_appearance, M, PATCH_SIZE);
  }

  warp.draw(color_image, feature.data(), PATCH_SIZE, color);
  */

  return 0;
}
