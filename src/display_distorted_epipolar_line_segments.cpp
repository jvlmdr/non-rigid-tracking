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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camera.hpp"
#include "multiview_track.hpp"
#include "quantize_ray.hpp"

#include "iterator_reader.hpp"
#include "camera_properties_reader.hpp"
#include "camera_pose_reader.hpp"
#include "read_image.hpp"
#include "read_lines.hpp"
#include "camera_properties_reader.hpp"

DEFINE_int32(thickness, 2, "Line thickness");

struct State {
  bool* have_point;
  cv::Point2d* point;
  int* selected_view;
  const std::vector<Camera>* cameras;
  std::map<double, cv::Point3d>* points;
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
    quantizeRay(*state.point, *state.cameras, params.view, 1., *state.points);
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
  std::map<double, cv::Point3d> points;

  // Program state for use by event handlers.
  State state;
  state.have_point = &have_point;
  state.point = &point;
  state.selected_view = &selected_view;
  state.cameras = &cameras;
  state.points = &points;

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
      a = std::log(points.begin()->first);
      b = std::log((--points.end())->first);
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
          std::map<double, cv::Point3d>::const_iterator point;
          for (point = points.begin(); point != points.end(); ++point) {
            int thickness = std::max(1, int(FLAGS_thickness / 2.));

            double lambda = point->first;
            double alpha = (std::log(lambda) - a) / (b - a);
            cv::Scalar color1(255, 0, 0);
            cv::Scalar color2(0, 0, 255);
            cv::Scalar color = alpha * color1 + (1 - alpha) * color2;

            const Camera& camera = cameras[view];
            cv::Point2d projection = camera.project(point->second);
            cv::circle(display, projection, thickness, color, -1);
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
