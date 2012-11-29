#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ceres/ceres.h>

#include "read_image.hpp"
#include "warp.hpp"
#include "similarity_warp.hpp"
#include "sift_position.hpp"
#include "flow.hpp"
#include "track_list.hpp"
#include "random_color.hpp"

#include "iterator_reader.hpp"
#include "sift_position_reader.hpp"
#include "sift_position_writer.hpp"
#include "track_list_writer.hpp"
#include "util.hpp"

DEFINE_int32(image_size, 512, "Maximum average dimension of image");
DEFINE_int32(radius, 8, "Half of [patch size - 1]");
DEFINE_double(mask_sigma, 4., "Sigma to use in mask");

// Optical flow settings (frame to frame).
const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-6;
const double GRADIENT_TOLERANCE = 1e-6;
const double PARAMETER_TOLERANCE = 1e-6;
const bool ITERATION_LIMIT_IS_FATAL = true;
const bool CHECK_CONDITION = true;
const double MAX_CONDITION = 1e3;

struct TrackedFeature {
  boost::shared_ptr<Warp> warp;
  cv::Mat reference;
};

////////////////////////////////////////////////////////////////////////////////

// enum
class Mode {
  public:
    static const int TRANSLATION = 0;
    static const int SIMILARITY = 1;
};

cv::Mat makeGaussian(double sigma, int width) {
  cv::Mat gaussian = cv::Mat_<double>(width, width);
  int radius = (width - 1) / 2;
  double sigma2 = sigma * sigma;

  for (int i = 0; i < width; i += 1) {
    double u = i - radius;
    for (int j = 0; j < width; j += 1) {
      double v = j - radius;
      double d2 = u * u + v * v;
      gaussian.at<double>(i, j) = std::exp(-d2 / (2. * sigma2));
    }
  }

  cv::Mat mask = cv::Mat_<double>::zeros(width, width);
  cv::Point center(radius, radius);
  cv::circle(mask, center, radius, 1., -1);
  cv::multiply(gaussian, mask, gaussian);

  return gaussian;
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Tracks points in a live demo";
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
}

int main(int argc, char** argv) {
  init(argc, argv);

  int diameter = FLAGS_radius * 2 + 1;

  // Open camera.
  cv::VideoCapture capture(0);
  CHECK(capture.isOpened()) << "Unable to open default camera";

  // Linear solver options.
  FlowOptions options;
  options.solver_options.linear_solver_type = ceres::DENSE_QR;
  options.solver_options.max_num_iterations = MAX_NUM_ITERATIONS;
  options.solver_options.function_tolerance = FUNCTION_TOLERANCE;
  options.solver_options.gradient_tolerance = GRADIENT_TOLERANCE;
  options.solver_options.parameter_tolerance = PARAMETER_TOLERANCE;
  options.check_condition = CHECK_CONDITION;
  options.max_condition = MAX_CONDITION;
  options.iteration_limit_is_fatal = ITERATION_LIMIT_IS_FATAL;
  options.interpolation = cv::INTER_AREA;

  // Construct mask.
  cv::Mat mask = makeGaussian(FLAGS_mask_sigma, diameter);

  bool exit = false;

  typedef std::list<TrackedFeature> FeatureList;
  FeatureList features;

  while (!exit) {
    cv::Mat color_image;
    capture >> color_image;
    int max_pixels = FLAGS_image_size * FLAGS_image_size;
    while (color_image.total() > max_pixels) {
      cv::pyrDown(color_image, color_image);
    }

    // Convert color to intensity.
    cv::Mat integer_gray_image;
    cv::cvtColor(color_image, integer_gray_image, CV_BGR2GRAY);
    // Convert to floating point in [0, 1].
    cv::Mat gray_image;
    integer_gray_image.convertTo(gray_image, cv::DataType<double>::type,
        1. / 255.);

    // Compute gradients using central difference.
    // Don't worry about smoothing, this will be done by downsampling.
    cv::Mat ddx;
    cv::Mat ddy;
    cv::Mat diff = (cv::Mat_<double>(1, 3) << -0.5, 0, 0.5);
    cv::Mat identity = (cv::Mat_<double>(1, 1) << 1);
    cv::sepFilter2D(gray_image, ddx, -1, diff, identity);
    cv::sepFilter2D(gray_image, ddy, -1, identity, diff);

    // Update each feature using the new image.
    FeatureList::iterator feature;
    for (feature = features.begin(); feature != features.end(); ++feature) {
      bool tracked = trackPatch(*feature->warp, feature->reference, gray_image,
          ddx, ddy, mask, options);
    }

    // Convert grayscale back to color for displaying.
    cv::Mat display;
    cv::cvtColor(integer_gray_image, display, CV_GRAY2BGR);

    // Display image.
    cv::imshow("image", display);
    char c = cv::waitKey(1000 / 30);

    if (c == 27) {
      exit = true;
    }
  }

  return 0;
}
