#include <string>
#include <queue>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include "multiview_track_list.hpp"
#include "track.hpp"
#include "random_color.hpp"
#include "match.hpp"

#include "read_image.hpp"
#include "read_lines.hpp"

#include "iterator_writer.hpp"
#include "image_point_writer.hpp"
#include "match_writer.hpp"

DEFINE_int32(radius, 5, "Base feature radius");
DEFINE_int32(line_thickness, 1, "Line thickness for drawing");

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

typedef std::map<int, cv::Point2d> Correspondence;

std::string makeViewFilename(const std::string& format,
                             const std::string& view) {
  return boost::str(boost::format(format) % view);
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Allows user to specify correspondences" << std::endl;
  usage << std::endl;
  usage << argv[0] << " image-format views num-frames keypoints-format matches" <<
      std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

struct State {
  bool observed;
  cv::Point2d position;

  State();
};

State::State() : observed(false), position() {}

void onMouse(int event, int x, int y, int, void* tag) {
  State& state = *static_cast<State*>(tag);

  if (event == cv::EVENT_LBUTTONDOWN) {
    state.observed = true;
    state.position = cv::Point2d(x, y);
  } else if (event == cv::EVENT_RBUTTONDOWN) {
    state.observed = false;
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  std::string views_file = argv[2];
  int num_frames = boost::lexical_cast<int>(argv[3]);
  std::string keypoints_format = argv[4];
  std::string matches_file = argv[5];
  int radius = FLAGS_radius;

  CHECK(num_frames > 0) << "Need at least one frame";

  bool ok;

  // Load names of views.
  std::vector<std::string> view_names;
  ok = readLines(views_file, view_names);
  CHECK(ok) << "Could not load view names";
  int num_views = view_names.size();

  // Load images at given time for all views.
  std::vector<cv::Mat> images;
  {
    for (int i = 0; i < num_views; i += 1) {
      cv::Mat image;
      std::string file = makeViewFilename(image_format, view_names[i]);
      ok = readColorImage(file, image);
      CHECK(ok) << "Could not load image";
      images.push_back(image);
    }
  }

  std::vector<State> states(num_views);

  // Open a window for each view.
  {
    for (int i = 0; i < num_views; i += 1) {
      cv::namedWindow(view_names[i]);
      cv::setMouseCallback(view_names[i], onMouse,
          static_cast<void*>(&states[i]));
    }
  }

  // Mark some tracks!
  std::vector<Correspondence> correspondences;

  std::vector<cv::Mat> display(num_views);
  std::vector<cv::Scalar> colors;
  colors.push_back(randomColor(BRIGHTNESS, SATURATION));
  bool exit = false;

  while (!exit) {
    for (int i = 0; i < num_views; i += 1) {
      display[i] = images[i].clone();
    }

    // Draw existing tracks on image.
    int num_points = correspondences.size();
    for (int i = 0; i < num_points; i += 1) {
      const Correspondence& correspondence = correspondences[i];

      Correspondence::const_iterator it;
      for (it = correspondence.begin(); it != correspondence.end(); ++it) {
        int v = it->first;
        const cv::Point2d& point = it->second;

        cv::Point2d pt1 = point - cv::Point2d(radius + 1, radius + 1);
        cv::Point2d pt2 = point + cv::Point2d(radius + 1, radius + 1);
        cv::rectangle(display[v], pt1, pt2, colors[i], FLAGS_line_thickness);
      }
    }

    // Draw current features on image.
    for (int i = 0; i < num_views; i += 1) {
      if (states[i].observed) {
        cv::Point2d point(states[i].position);
        cv::Point2d pt1 = point - cv::Point2d(radius + 1, radius + 1);
        cv::Point2d pt2 = point + cv::Point2d(radius + 1, radius + 1);
        cv::rectangle(display[i], pt1, pt2, colors.back(),
            FLAGS_line_thickness * 2);
      }
    }


    // Show image.
    for (int i = 0; i < num_views; i += 1) {
      cv::imshow(view_names[i], display[i]);
    }

    char c = cv::waitKey(30);

    if (c == 27) {
      exit = true;
    } else if (c == '\r' || c == '\n') {
      // Build single correspondence.
      std::map<int, cv::Point2d> correspondence;

      for (int i = 0; i < num_views; i += 1) {
        if (states[i].observed) {
          correspondence[i] = states[i].position;
        }
      }

      if (!correspondence.empty()) {
        correspondences.push_back(correspondence);
        colors.push_back(randomColor(BRIGHTNESS, SATURATION));
        states.assign(num_views, State());

        LOG(INFO) << correspondences.size() << " correspondences";
      }
    }
  }

  // Convert to list of keypoints and matches.
  std::vector<std::vector<cv::Point2d> > points(num_views);
  // Assume two views!! No time!! CVPR!!
  CHECK(num_views == 2);
  std::vector<Match> matches;

  // Draw existing tracks on image.
  {
    int num_points = correspondences.size();

    for (int i = 0; i < num_points; i += 1) {
      const Correspondence& correspondence = correspondences[i];

      Correspondence::const_iterator it;
      for (it = correspondence.begin(); it != correspondence.end(); ++it) {
        int view = it->first;
        const cv::Point2d& point = it->second;

        points[view].push_back(point);
      }

      if (correspondence.size() == 2) {
        matches.push_back(Match(points[0].size() - 1, points[1].size() - 1));
      }
    }
  }

  // Save out lists of keypoints.
  ImagePointWriter<double> point_writer;
  for (int i = 0; i < num_views; i += 1) {
    std::string file = makeViewFilename(keypoints_format, view_names[i]);
    ok = saveList(file, points[i], point_writer);
    CHECK(ok) << "Could not save keypoints";
  }

  // Save out lists of matches.
  MatchWriter match_writer;
  ok = saveList(matches_file, matches, match_writer);
  CHECK(ok) << "Could not save matches";

  return 0;
}
