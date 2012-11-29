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
#include "translation_warp.hpp"
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

DEFINE_int32(max_image_size, 512, "Maximum average dimension of image");
DEFINE_int32(radius, 8, "Half of [patch size - 1]");
DEFINE_double(mask_sigma, 4., "Sigma to use in mask");
DEFINE_double(min_scale, 1., "Minimum warp scale");

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
  cv::Mat appearance;
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

struct State {
  const TranslationWarper* translation_warper;
  const SimilarityWarper* similarity_warper;
  std::list<TrackedFeature>* features;
  const cv::Mat* image;
  int radius;
  const FlowOptions* options;
};

void onMouse(int event, int x, int y, int, void* tag) {
  State& state = *static_cast<State*>(tag);

  if (event == CV_EVENT_LBUTTONDOWN || event == CV_EVENT_RBUTTONDOWN) {
    // Clicked! Add a feature.
    TrackedFeature feature;

    if (event == CV_EVENT_LBUTTONDOWN) {
      // Left-clicked. Add a translation feature.
      feature.warp.reset(
          new TranslationWarp(x, y, *state.translation_warper));
    } else {
      // Right-clicked. Add a similarity feature.
      feature.warp.reset(
          new SimilarityWarp(x, y, std::log(2.), 0., *state.similarity_warper));
    }

    // Extract initial appearance.
    int diameter = 2 * state.radius + 1;
    samplePatch(*feature.warp, *state.image, feature.appearance, diameter,
        false, state.options->interpolation);

    state.features->push_back(feature);
  }
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

  // Set up different warps.
  TranslationWarper::CostFunction translation_cost(
      new TranslationWarpFunction());
  TranslationWarper translation_warper(translation_cost);

  SimilarityWarper::CostFunction similarity_cost(new SimilarityWarpFunction());
  SimilarityWarper similarity_warper(similarity_cost, FLAGS_min_scale);

  bool exit = false;
  char keypress = 0;

  typedef std::list<TrackedFeature> FeatureList;
  FeatureList features;

  cv::Mat image;

  State state;
  state.translation_warper = &translation_warper;
  state.similarity_warper = &similarity_warper;
  state.features = &features;
  state.image = &image;
  state.radius = FLAGS_radius;
  state.options = &options;

  cv::namedWindow("video");
  cv::setMouseCallback("video", onMouse, &state);

  while (!exit) {
    cv::Mat color_image;
    capture >> color_image;
    int max_pixels = FLAGS_max_image_size * FLAGS_max_image_size;
    while (color_image.total() > max_pixels) {
      cv::pyrDown(color_image, color_image);
    }

    // Convert color to intensity.
    cv::Mat integer_gray_image;
    cv::cvtColor(color_image, integer_gray_image, CV_BGR2GRAY);
    // Convert to floating point in [0, 1].
    integer_gray_image.convertTo(image, cv::DataType<double>::type, 1. / 255.);

    // Compute gradients using central difference.
    // Don't worry about smoothing, this will be done by downsampling.
    cv::Mat ddx;
    cv::Mat ddy;
    cv::Mat diff = (cv::Mat_<double>(1, 3) << -0.5, 0, 0.5);
    cv::Mat identity = (cv::Mat_<double>(1, 1) << 1);
    cv::sepFilter2D(image, ddx, -1, diff, identity);
    cv::sepFilter2D(image, ddy, -1, identity, diff);

    // Track features from the previous image.
    {
      FeatureList::iterator feature = features.begin();
      while (feature != features.end()) {
        bool tracked = trackPatch(*feature->warp, feature->appearance, image,
            ddx, ddy, mask, options);

        if (!tracked) {
          // Failed to track. Erase feature and move on.
          features.erase(feature++);
        } else {
          // Move to next.
          ++feature;
        }
      }
    }

    // Convert grayscale back to color for displaying.
    cv::Mat display;
    cv::cvtColor(integer_gray_image, display, CV_GRAY2BGR);

    // Draw features.
    LOG(INFO) << features.size() << " features";
    {
      FeatureList::const_iterator feature;
      for (feature = features.begin(); feature != features.end(); ++feature) {
        cv::Scalar color(200, 0, 0);
        feature->warp->draw(display, FLAGS_radius, color, 1);
      }
    }

    // Display image.
    cv::imshow("video", display);
    keypress = cv::waitKey(1000 / 30);

    if (keypress == 27) {
      exit = true;
    }
  }

  return 0;
}
