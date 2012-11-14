#include <string>
#include <cstdlib>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/scoped_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sift_position.hpp"
#include "draw_sift_position.hpp"

#include "sift_position_reader.hpp"
#include "iterator_reader.hpp"
#include "read_image.hpp"

#include "default_writer.hpp"
#include "iterator_writer.hpp"

DEFINE_int32(radius, 5, "Base radius of features");
DEFINE_int32(line_thickness, 1, "Line thickness");

typedef std::vector<SiftPosition> FeatureSet;

void drawFeatures(cv::Mat& image,
                  const FeatureSet& features,
                  const cv::Scalar& color) {
  FeatureSet::const_iterator feature;
  for (feature = features.begin(); feature != features.end(); ++feature) {
    drawSiftPosition(image, *feature, color, FLAGS_line_thickness);
  }
}

struct State {
  cv::Point position;
  boost::scoped_ptr<cv::Point> corner1;
  boost::scoped_ptr<cv::Point> corner2;
};

void onMouse(int event, int x, int y, int, void* tag) {
  State& state = *static_cast<State*>(tag);

  // Set position.
  state.position = cv::Point(x, y);

  if (event == cv::EVENT_LBUTTONDOWN) {
    // Set first corner.
    state.corner1.reset(new cv::Point(x, y));
    state.corner2.reset();
  } else if (event == cv::EVENT_LBUTTONUP) {
    // Set second corner.
    state.corner2.reset(new cv::Point(x, y));
  }
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Allows the user to choose a subset of keypoints" << std::endl;
  usage << std::endl;
  usage << argv[0] << " keypoints image indices" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  bool ok;

  init(argc, argv);
  std::string keypoints_file = argv[1];
  std::string image_file = argv[2];
  std::string subset_file = argv[3];

  // Load keypoints.
  std::vector<SiftPosition> keypoints;
  SiftPositionReader feature_reader;
  ok = loadList(keypoints_file, keypoints, feature_reader);
  CHECK(ok) << "Could not load keypoints";
  int num_keypoints = keypoints.size();
  LOG(INFO) << "Loaded " << num_keypoints << " keypoints";

  // Load image.
  cv::Mat image;
  readColorImage(image_file, image);

  bool exit = false;
  bool save = false;
  State state;

  cv::namedWindow("keypoints");
  cv::setMouseCallback("keypoints", onMouse, static_cast<void*>(&state));

  boost::scoped_ptr<cv::Rect> bounds;
  std::vector<int> subset;

  while (!exit) {
    bool have_rect = (state.corner1 && state.corner2);

    subset.clear();
    std::vector<SiftPosition> inside;
    std::vector<SiftPosition> outside;

    // If rectangle is set, divide features into two sets.
    if (state.corner1) {
      if (state.corner2) {
        bounds.reset(new cv::Rect(*state.corner1, *state.corner2));
      } else {
        bounds.reset(new cv::Rect(*state.corner1, state.position));
      }

      std::vector<SiftPosition>::const_iterator feature;
      int index = 0;

      for (feature = keypoints.begin(); feature != keypoints.end(); ++feature) {
        if (bounds->contains(feature->point())) {
          subset.push_back(index);
          inside.push_back(*feature);
        } else {
          outside.push_back(*feature);
        }

        index += 1;
      }
    } else {
      bounds.reset();
      std::copy(keypoints.begin(), keypoints.end(),
          std::back_inserter(outside));
    }

    // Draw features.
    cv::Mat display = image.clone();
    drawFeatures(display, inside, cv::Scalar(0x66, 0xCC, 0x00));
    drawFeatures(display, outside, cv::Scalar(0x00, 0xCC, 0xFF));
    if (bounds) {
      cv::rectangle(display, *bounds, cv::Scalar(0x00, 0x00, 0xCC), 1);
    }

    if (!(state.corner1 && !state.corner2)) {
      cv::line(display, cv::Point(state.position.x, 0),
          cv::Point(state.position.x, image.rows), cv::Scalar(0x00, 0x00, 0xCC),
          1);
      cv::line(display, cv::Point(0, state.position.y),
          cv::Point(image.cols, state.position.y), cv::Scalar(0x00, 0x00, 0xCC),
          1);
    }

    // Show image.
    cv::imshow("keypoints", display);
    char c = cv::waitKey(20);

    if (c == '\r' || c == '\n') {
      if (have_rect) {
        exit = true;
        save = true;
      }
    } else if (c == 27) {
      if (have_rect) {
        state.corner1.reset();
        state.corner2.reset();
      } else {
        exit = true;
      }
    }
  }

  if (save) {
    DefaultWriter<int> index_writer;
    saveList(subset_file, subset, index_writer);

    LOG(INFO) << "Selected " << subset.size() << " / " << num_keypoints <<
        " keypoints";
  }

  return 0;
}
