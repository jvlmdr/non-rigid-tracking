#include <string>
#include <vector>
#include <list>
#include <set>
#include <algorithm>
#include <sstream>
#include <cstdlib>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/math/tools/roots.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camera.hpp"
#include "distortion.hpp"
#include "multiview_track.hpp"
#include "util.hpp"

#include "iterator_reader.hpp"
#include "camera_properties_reader.hpp"
#include "camera_pose_reader.hpp"
#include "read_image.hpp"
#include "read_lines.hpp"
#include "camera_properties_reader.hpp"

DEFINE_int32(thickness, 2, "Line thickness");

double EPSILON = 1e-6;

////////////////////////////////////////////////////////////////////////////////

double errorInDistanceFromPoint(cv::Point2d y,
                                double lambda,
                                const cv::Mat A,
                                const cv::Mat B,
                                double delta,
                                const CameraProperties& intrinsics) {
  cv::Mat X = A + lambda * B;
  cv::Point2d x = imagePointFromHomogeneous(X);
  x = intrinsics.distortAndUncalibrate(x);
  return cv::norm(x - y) - delta;
}

struct ProjectiveLine {
  cv::Mat A;
  cv::Mat B;
};

// All you need to quantize a 3D ray in one view.
struct Quantizer {
  // Index for establishing uniqueness and order in a set.
  int index;
  // Parameters of 2D line (calibrated and undistorted).
  ProjectiveLine line;
  // Interval of ray which is in front of camera.
  double lambda_min;
  double lambda_max;
  // Position of current lambda.
  // TODO: This should be a map not a set!
  mutable cv::Point2d x;

  CameraProperties intrinsics;

  bool operator<(const Quantizer& other) const {
    return index < other.index;
  }
};

double maximumErrorInDistanceFromPoint(const std::set<Quantizer>& views,
                                       double lambda,
                                       double delta) {
  double max = -std::numeric_limits<double>::infinity();

  std::set<Quantizer>::const_iterator view;
  for (view = views.begin(); view != views.end(); ++view) {
    CHECK(view->lambda_min <= lambda);
    CHECK(lambda <= view->lambda_max);

    // Compute position on line.
    cv::Mat X = view->line.A + lambda * view->line.B;
    cv::Point2d x = imagePointFromHomogeneous(X);
    x = view->intrinsics.distortAndUncalibrate(x);

    // Compute error in distance from previous position.
    double e = cv::norm(x - view->x) - delta;

    if (e > max) {
      max = e;
    }
  }

  return max;
}

double maxLambdaMin(double lambda_min, const Quantizer& quantizer) {
  return std::max(lambda_min, quantizer.lambda_min);
}

bool lambdaMinGreaterThan(const Quantizer& quantizer, double lambda) {
  return quantizer.lambda_min > lambda;
}

double computeLambdaMax(const cv::Mat& A,
                        const cv::Mat& B,
                        const CameraProperties& intrinsics,
                        cv::Point2d& x) {
  double a3 = A.at<double>(2, 0);
  double b3 = B.at<double>(2, 0);

  double lambda_max;

  // Set lambda_max and lambda depending on destination of ray.
  if (b3 < 0) {
    // Ray goes to infinity in front of camera. There is a vanishing point.
    DLOG(INFO) << "Ray ends in front of camera";
    // No upper bound on lambda.
    lambda_max = std::numeric_limits<double>::infinity();

    // Compute vanishing point.
    x = imagePointFromHomogeneous(B);
    x = intrinsics.distortAndUncalibrate(x);
  } else {
    // Ray goes to infinity behind camera, crossing image plane. There is no
    // vanishing point. However, under distortion, a 2D point at infinity
    // will still have a finite position.
    DLOG(INFO) << "Ray ends behind camera";
    lambda_max = -a3 / b3;
    CHECK(lambda_max > 0);

    // Find position of point at infinity after distortion.
    cv::Mat X = A + lambda_max * B;
    x = cv::Point2d(X.at<double>(0, 0), X.at<double>(1, 0));
    x = distortPointAtInfinity(x, intrinsics.distort_w);
    x = intrinsics.uncalibrate(x);

    // Shrink by some epsilon to allow non-strict inequality.
    lambda_max *= (1. - EPSILON);
  }

  return lambda_max;
}

double computeLambdaMin(const cv::Mat& A,
                        const cv::Mat& B,
                        const CameraProperties& intrinsics,
                        cv::Point2d& x) {
  double a3 = A.at<double>(2, 0);
  double b3 = B.at<double>(2, 0);

  double lambda_min;

  // Set lambda_min depending on position of camera center.
  if (a3 < 0) {
    // Ray starts in front of camera. 2D line starts at a finite coordinate.
    DLOG(INFO) << "Ray starts in front of camera";
    lambda_min = 0;

    // Compute epipole.
    x = imagePointFromHomogeneous(A);
    x = intrinsics.distortAndUncalibrate(x);
  } else {
    // Ray starts behind camera. 2D line starts at infinity.
    DLOG(INFO) << "Ray starts behind camera";
    lambda_min = -a3 / b3;
    CHECK(lambda_min > 0);

    // Find position of point at infinity after distortion.
    cv::Mat X = A + lambda_min * B;
    x = cv::Point2d(X.at<double>(0, 0), X.at<double>(1, 0));
    x = distortPointAtInfinity(x, intrinsics.distort_w);
    x = intrinsics.uncalibrate(x);

    // Grow by some epsilon to allow non-strict inequality.
    lambda_min *= (1. + EPSILON);
  }

  return lambda_min;
}

void quantizeRay(const cv::Point2d& projection,
                 const std::vector<Camera>& cameras,
                 int selected,
                 double delta,
                 std::vector<double>& lambdas,
                 MultiviewTrack<cv::Point2d>& lines) {
  int num_views = cameras.size();

  lambdas.clear();
  lines = MultiviewTrack<cv::Point2d>(num_views);

  // Solutions parametrized by 3D line c + lambda v, lambda >= 0.
  const Camera& camera = cameras[selected];
  cv::Point3d c = camera.extrinsics().center;
  cv::Point2d w = camera.intrinsics().calibrateAndUndistort(projection);
  cv::Point3d v = camera.extrinsics().directionOfRayThrough(w);

  cv::Mat C = worldPointToHomogeneous(c);
  cv::Mat V = worldPointToHomogeneous(v, 0);

  // Initialize Quantizer for each view.
  std::set<Quantizer> pending;

  {
    std::vector<Camera>::const_iterator other;
    int index = 0;

    for (other = cameras.begin(); other != cameras.end(); ++other) {
      if (index != selected) {
        // Find 2D projective line.
        cv::Mat P(other->extrinsics().matrix());
        ProjectiveLine line;
        line.A = P * C;
        line.B = P * V;

        // Check whether each point is in front of or behind the camera.
        double a3 = line.A.at<double>(2, 0);
        double b3 = line.B.at<double>(2, 0);

        if (a3 > 0 && b3 > 0) {
          // Entire ray is behind camera.
          DLOG(INFO) << "Ray is not observed";
        } else {
          double lambda_min;
          double lambda_max;
          cv::Point2d x;

          lambda_max = computeLambdaMax(line.A, line.B, other->intrinsics(), x);
          cv::Point2d y;
          lambda_min = computeLambdaMin(line.A, line.B, other->intrinsics(), y);

          Quantizer view;
          view.index = index;
          view.line = line;
          view.lambda_min = lambda_min;
          view.lambda_max = lambda_max;
          view.x = x;
          view.intrinsics = other->intrinsics();

          pending.insert(view);
        }
      }

      index += 1;
    }
  }

  // Find initial lambda.
  double lambda = 0;

  for (std::set<Quantizer>::const_iterator view = pending.begin();
       view != pending.end();
       ++view) {
    if (view->lambda_max < std::numeric_limits<double>::infinity()) {
      // lambda_max is finite.
      lambda = std::max(lambda, view->lambda_max);
    } else {
      // lambda_max is infinite.
      // Find a lambda which is big enough.
      bool big_enough = false;

      // Ensure that we don't evaluate below lambda_min.
      lambda = std::max(lambda, view->lambda_min);

      while (!big_enough) {
        double error = errorInDistanceFromPoint(view->x, lambda, view->line.A,
            view->line.B, delta, view->intrinsics);

        if (error < 0) {
          big_enough = true;
        } else {
          if (lambda == 0) {
            lambda = std::max(lambda, 1.);
          } else {
            lambda *= 2;
          }
        }
      }
    }
  }

  bool converged = false;
  int t = 0;

  std::set<Quantizer> active;

  while (!converged) {
    // Move points from pending to active set for which lambda <= lambda_max.
    {
      std::set<Quantizer>::iterator view = pending.begin();
      while (view != pending.end()) {
        // Is the domain of the function within the bisection range?
        if (lambda <= view->lambda_max) {
          active.insert(*view);
          pending.erase(view++);
        } else {
          ++view;
        }
      }
    }

    // Remove points from active set for which lambda < lambda_min.
    {
      std::set<Quantizer> valid;
      std::remove_copy_if(active.begin(), active.end(),
          std::inserter(valid, valid.begin()),
          boost::bind(lambdaMinGreaterThan, _1, lambda));
      active.swap(valid);
    }

    CHECK(!(active.empty() && !pending.empty()));

    // Update positions.
    std::set<Quantizer>::iterator view;
    for (view = active.begin(); view != active.end(); ++view) {
      cv::Mat X = view->line.A + lambda * view->line.B;
      cv::Point2d x = imagePointFromHomogeneous(X);
      x = view->intrinsics.distortAndUncalibrate(x);
      view->x = x;
    }

    // Remove points which do not have a solution in (lambda_min(i), lambda).
    {
      std::set<Quantizer>::iterator view = active.begin();
      while (view != active.end()) {
        // Is there a sign change across the interval?
        double f_max = errorInDistanceFromPoint(view->x, view->lambda_min,
            view->line.A, view->line.B, delta, view->intrinsics);

        // We know that f(lambda_max) < 0, so f(lower) should be > 0.
        if (f_max < 0) {
          active.erase(view++);
        } else {
          ++view;
        }
      }
    }

    if (active.empty() && pending.empty()) {
      converged = true;
    } else {
      CHECK(!active.empty());

      // Update lower bound to be maximum lambda_min over active set.
      double lower = 0;
      lower = std::accumulate(active.begin(), active.end(), lower, maxLambdaMin);

      // Find lambda which gives point at most delta pixels away from x.
      double old_lambda = lambda;
      std::pair<double, double> interval = boost::math::tools::bisect(
          boost::bind(maximumErrorInDistanceFromPoint, active, _1, delta),
          lower, lambda, boost::math::tools::eps_tolerance<double>(16));
      lambda = interval.second;

      // Guard against limit cycles.
      if (t > 0) {
        CHECK(lambda != old_lambda) << "Encountered limit cycle";
      }

      // Add points to tracks.
      lambdas.push_back(lambda);
      std::set<Quantizer>::iterator view;
      for (view = active.begin(); view != active.end(); ++view) {
        cv::Mat X = view->line.A + lambda * view->line.B;
        cv::Point2d x = imagePointFromHomogeneous(X);
        x = view->intrinsics.distortAndUncalibrate(x);
        lines.view(view->index)[t] = x;
      }

      t += 1;
    }
  }

  LOG(INFO) << "Quantized ray into " << lambdas.size() << " positions";
}

////////////////////////////////////////////////////////////////////////////////

struct State {
  bool* have_point;
  cv::Point2d* point;
  int* selected_view;
  const std::vector<Camera>* cameras;
  std::vector<double>* lambdas;
  MultiviewTrack<cv::Point2d>* lines;
};

struct Params {
  State* state;
  int view;
};

void onMouse(int event, int x, int y, int, void* tag) {
  Params& params = *static_cast<Params*>(tag);
  State& state = *params.state;

  if (event == CV_EVENT_LBUTTONDOWN) {
    *state.point = cv::Point2d(x, y);
    *state.have_point = true;
    *state.selected_view = params.view;

    // Find each line.
    quantizeRay(*state.point, *state.cameras, params.view, 1., *state.lambdas,
        *state.lines);
  } else if (event == CV_EVENT_RBUTTONDOWN) {
    *state.have_point = false;
  }
}

////////////////////////////////////////////////////////////////////////////////

std::string makeViewFilename(const std::string& format,
                             const std::string& name) {
  return boost::str(boost::format(format) % name);
}

std::string makeImageFilename(const std::string& format,
                              const std::string& view,
                              int time) {
  return boost::str(boost::format(format) % view % (time + 1));
}

Camera loadCamera(const std::string& view,
                  const std::string& extrinsics_format,
                  const std::string& intrinsics_format) {
  bool ok;

  CameraProperties intrinsics;
  std::string intrinsics_file = makeViewFilename(intrinsics_format, view);
  CameraPropertiesReader intrinsics_reader;
  ok = load(intrinsics_file, intrinsics, intrinsics_reader);
  CHECK(ok) << "Could not load intrinsics";

  CameraPose extrinsics;
  std::string extrinsics_file = makeViewFilename(extrinsics_format, view);
  CameraPoseReader extrinsics_reader;
  ok = load(extrinsics_file, extrinsics, extrinsics_reader);
  CHECK(ok) << "Could not load extrinsics";

  return Camera(intrinsics, extrinsics);
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Visualizes distorted epipolar line segments" << std::endl;
  usage << std::endl;
  usage << argv[0] << " extrinsics-format intrinsics-format image-format views "
      "num-frames" << std::endl;
  usage << std::endl;
  usage << "Parameters:" << std::endl;
  usage << "extrinsics-format -- e.g. extrinsics/%s.yaml" << std::endl;
  usage << "intrinsics-format -- e.g. intrinsics/%s.yaml" << std::endl;
  usage << "image-format -- e.g. images/%s/%07d.png" << std::endl;
  usage << "view -- Text file whose lines are the view names" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  bool ok;

  init(argc, argv);
  std::string extrinsics_format = argv[1];
  std::string intrinsics_format = argv[2];
  std::string image_format = argv[3];
  std::string views_file = argv[4];
  int num_frames = boost::lexical_cast<int>(argv[5]);

  // Load names of views.
  std::vector<std::string> view_names;
  ok = readLines(views_file, view_names);
  CHECK(ok) << "Could not load view names";
  int num_views = view_names.size();
  LOG(INFO) << "Matching to " << num_views << " views";

  // Load cameras.
  std::vector<Camera> cameras;
  std::transform(view_names.begin(), view_names.end(),
      std::back_inserter(cameras),
      boost::bind(loadCamera, _1, extrinsics_format, intrinsics_format));
  LOG(INFO) << "Loaded " << cameras.size() << " cameras";

  // Play movie.
  bool exit = false;
  bool paused = true;
  int time = 0;

  bool have_point = false;
  cv::Point2d point;
  int selected_view = 0;
  std::vector<double> lambdas;
  MultiviewTrack<cv::Point2d> lines;

  // Program state for use by event handlers.
  State state;
  state.have_point = &have_point;
  state.point = &point;
  state.selected_view = &selected_view;
  state.cameras = &cameras;
  state.lambdas = &lambdas;
  state.lines = &lines;

  // Open one window for each view.
  std::list<Params> params;
  for (int i = 0; i < num_views; i += 1) {
    Params view_params;
    view_params.state = &state;
    view_params.view = i;
    params.push_back(view_params);

    const std::string& view_name = view_names[i];
    cv::namedWindow(view_name);
    cv::setMouseCallback(view_name, onMouse, &params.back());
  }

  while (!exit) {
    cv::Mat image;
    cv::Mat display;

    double a = 0;
    double b = 0;
    if (have_point) {
      a = std::log(lambdas.back());
      b = std::log(lambdas.front());
    }

    // Show image in each window.
    for (int view = 0; view < num_views; view += 1) {
      // Load image.
      const std::string &view_name = view_names[view];
      std::string file = makeImageFilename(image_format, view_name, time);
      ok = readGrayImage(file, image);
      CHECK(ok) << "Could not load image";
      cv::cvtColor(image, display, CV_GRAY2BGR);

      if (have_point) {
        if (view == selected_view) {
          // Show the point in the selected image.
          cv::circle(display, point, 4, cv::Scalar(0, 0, 255),
              FLAGS_thickness);
          cv::circle(display, point, 16, cv::Scalar(0, 0, 255),
              FLAGS_thickness);
          cv::circle(display, point, 64, cv::Scalar(0, 0, 255),
              FLAGS_thickness);
        } else {
          // Show line in other images.
          const Track<cv::Point2d>& line = lines.view(view);
          Track<cv::Point2d>::const_iterator point;
          for (point = line.begin(); point != line.end(); ++point) {
            double lambda = lambdas[point->first];
            double alpha = (std::log(lambda) - a) / (b - a);
            LOG(INFO) << "lambda = " << lambda << ", alpha = " << alpha;
            cv::Scalar color1(255, 0, 0);
            cv::Scalar color2(0, 0, 255);
            cv::Scalar color = alpha * color1 + (1 - alpha) * color2;
            int thickness = std::max(1, int(FLAGS_thickness / 2.));
            cv::circle(display, point->second, thickness, color, -1);
          }
        }
      }

      cv::imshow(view_name, display);
    }

    char c = cv::waitKey(std::ceil(1000. / 30));

    if (c == 27) {
      exit = true;
    } else if (c == ' ') {
      paused = !paused;
    }

    if (!paused) {
      time = (time + 1) % num_frames;
    }
  }

  return 0;
}
