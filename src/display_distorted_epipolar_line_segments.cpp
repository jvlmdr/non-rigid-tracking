#include <string>
#include <vector>
#include <list>
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
#include "util.hpp"

#include "iterator_reader.hpp"
#include "camera_properties_reader.hpp"
#include "camera_pose_reader.hpp"
#include "read_image.hpp"
#include "read_lines.hpp"
#include "camera_properties_reader.hpp"

DEFINE_int32(thickness, 2, "Line thickness");

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

bool toleranceIsOk(double x, double y, double epsilon) {
  return std::abs(x - y) < epsilon;
}

void findExtentOfRay(const cv::Point2d& projection,
                     const std::vector<Camera>& cameras,
                     int view,
                     std::deque<std::vector<cv::Point2d> >& lines) {
  const Camera& camera = cameras[view];

  cv::Point2d w = camera.intrinsics().calibrateAndUndistort(projection);
  cv::Point3d c = camera.extrinsics().center;

  // Find vector in nullspace of linear projection system, A = R_xy - w R_z.
  cv::Mat R(camera.extrinsics().rotation);
  cv::Mat A = R.rowRange(0, 2) - cv::Mat(w) * R.rowRange(2, 3);
  // 1D nullspace found trivially by cross-product.
  // Take negative i x j because z < 0 is in front of camera.
  cv::Mat V = -A.row(0).t().cross(A.row(1).t());
  cv::Point3d v(V.at<double>(0, 0), V.at<double>(0, 1), V.at<double>(0, 2));

  lines.clear();

  // Space of solutions parametrized by 3D line c + lambda v, with lambda >= 0.
  // For each image, find intersections of the projected line with the outer
  // radius of the lens distortion.
  std::vector<Camera>::const_iterator other;
  int index = 0;

  for (other = cameras.begin(); other != cameras.end(); ++other) {
    std::vector<cv::Point2d> line;

    if (index != view) {
      cv::Mat P(other->extrinsics().matrix());

      cv::Mat C = worldPointToHomogeneous(c);
      cv::Mat V = worldPointToHomogeneous(v, 0);
      cv::Mat A = P * C;
      cv::Mat B = P * V;

      // Assume that line has a vanishing point (b is not at infinity).
      // TODO: Cope with non-vanishing-point case (b at infinity).
      //cv::Point2d a = imagePointFromHomogeneous(A);
      //CHECK(B.at<double>(2, 0) != 0);
      //cv::Point2d b = imagePointFromHomogeneous(B);

      double a3 = A.at<double>(2, 0);
      double b3 = B.at<double>(2, 0);

      if (a3 > 0 && b3 > 0) {
        // Entire ray is behind camera.
        DLOG(INFO) << "Ray is not observed";
        continue;
      }

      double delta = 1;

      double lambda;
      cv::Point2d x;
      double lambda_min;

      if (a3 < 0) {
        // Ray starts in front of camera. Line starts at a finite coordinate.
        DLOG(INFO) << "Ray starts in front of camera";
        lambda_min = 0;
      } else {
        // Ray starts behind camera. Line starts at infinity.
        DLOG(INFO) << "Ray starts behind camera";
        lambda_min = -a3 / b3;
        CHECK(lambda_min > 0);
        lambda_min = lambda_min * (1. + 1e-6);
      }
      DLOG(INFO) << "lambda_min => " << lambda_min;

      if (b3 < 0) {
        // Ray goes to infinity in front of camera. There is a vanishing point.
        DLOG(INFO) << "Ray ends in front of camera";
        CHECK(B.at<double>(2, 0) != 0);
        x = imagePointFromHomogeneous(B);
        x = other->intrinsics().distortAndUncalibrate(x);

        // We can't use lambda = infinity for bisection.
        // Find a lambda which is big enough.
        lambda = 1.;
        bool found = false;
        while (!found) {
          double error = errorInDistanceFromPoint(x, lambda, A, B, delta,
              other->intrinsics());
          if (error < 0) {
            found = true;
          }
          lambda *= 2;
        }
      } else {
        // Ray goes to infinity behind camera, crossing image plane.
        // There is no vanishing point. However, under distortion, a 2D point at
        // infinity will still have a finite position.
        DLOG(INFO) << "Ray ends behind camera";
        lambda = -a3 / b3;
        CHECK(lambda > 0);
        lambda = lambda * (1 - 1e-6);

        cv::Mat X = A + lambda * B;
        x = imagePointFromHomogeneous(X);
        x = other->intrinsics().distortAndUncalibrate(x);
      }
      DLOG(INFO) << "lambda_max => " << lambda;
      DLOG(INFO) << "x(lambda_max) => " << x;

      bool converged = false;
      bool first = true;

      while (!converged) {
        // Check there is a point on the line at least delta pixels away from x.
        double f_max = errorInDistanceFromPoint(x, lambda_min, A, B, delta,
              other->intrinsics());

        if (f_max < 0) {
          // No solution is possible.
          converged = true;
        } else {
          double old_lambda = lambda;

          // Find lambda which gives point delta pixels away from x.
          std::pair<double, double> interval = boost::math::tools::bisect(
                boost::bind(errorInDistanceFromPoint, x, _1, A, B, delta,
                  other->intrinsics()),
                lambda_min, lambda,
                boost::math::tools::eps_tolerance<double>(16));
          lambda = interval.second;

          // Guard against limit cycles.
          if (!first) {
            CHECK(lambda != old_lambda) << "Entered limit cycle";
          }

          // Update position.
          cv::Mat X = A + lambda * B;
          x = imagePointFromHomogeneous(X);
          x = other->intrinsics().distortAndUncalibrate(x);

          line.push_back(x);
        }

        first = false;
      }

      DLOG(INFO) << "Quantized ray into " << line.size() << " positions";
    }

    lines.push_back(std::vector<cv::Point2d>());
    lines.back().swap(line);

    index += 1;
  }
}

////////////////////////////////////////////////////////////////////////////////

struct State {
  bool* have_point;
  cv::Point2d* point;
  int* selected_view;
  const std::vector<Camera>* cameras;
  std::deque<std::vector<cv::Point2d> >* lines;
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
    findExtentOfRay(*state.point, *state.cameras, params.view, *state.lines);
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
  std::deque<std::vector<cv::Point2d> > lines;

  // Program state for use by event handlers.
  State state;
  state.have_point = &have_point;
  state.point = &point;
  state.selected_view = &selected_view;
  state.cameras = &cameras;
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
          const std::vector<cv::Point2d>& line = lines[view];
          std::vector<cv::Point2d>::const_iterator point;
          for (point = line.begin(); point != line.end(); ++point) {
            int thickness = std::max(1, int(FLAGS_thickness / 2.));
            cv::circle(display, *point, thickness, cv::Scalar(0, 0, 255), -1);
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
