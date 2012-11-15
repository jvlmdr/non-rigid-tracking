#include <string>
#include <cstdlib>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "track_list.hpp"
#include "scale_space_position.hpp"
#include "scale_space_feature_drawer.hpp"
#include "random_color.hpp"

#include "track_list_reader.hpp"
#include "default_reader.hpp"
#include "read_lines.hpp"
#include "iterator_reader.hpp"
#include "scale_space_position_reader.hpp"
#include "read_image.hpp"

DEFINE_int32(radius, 5, "Base feature radius");
DEFINE_int32(line_thickness, 2, "Line thickness for drawing");

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const double LINE_THICKNESS = 2;

std::string makeImageFilename(const std::string& format, int time) {
  return boost::str(boost::format(format) % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Allows the user to clean up tracks" << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks image-format num-frames" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

struct State {
  cv::Point position;
  boost::scoped_ptr<cv::Point> corner;
  boost::scoped_ptr<cv::Rect> rect;
};

void onMouse(int event, int x, int y, int, void* tag) {
  State& state = *static_cast<State*>(tag);

  // Set position.
  state.position = cv::Point(x, y);

  if (event == cv::EVENT_LBUTTONDOWN) {
    // Set first corner.
    state.corner.reset(new cv::Point(x, y));
    state.rect.reset();
  } else if (event == cv::EVENT_LBUTTONUP) {
    // Set rectangle using second corner.
    cv::Point other(x, y);
    state.rect.reset(new cv::Rect(*state.corner, other));
    // Clear first corner.
    state.corner.reset();
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];
  int num_frames = boost::lexical_cast<int>(argv[3]);

  bool ok;

  // Load tracks.
  // TODO: Feature needs to have draw(), x and y position and write().
  TrackList<ScaleSpacePosition> tracks;
  ScaleSpacePositionReader feature_reader;
  ok = loadTrackList(tracks_file, tracks, feature_reader);
  CHECK(ok) << "Could not load tracks";

  int num_tracks = tracks.size();
  LOG(INFO) << "Loaded " << num_tracks << " tracks";

  // Generate a color for each track.
  std::vector<cv::Scalar> colors;
  for (int i = 0; i < num_tracks; i += 1) {
    colors.push_back(randomColor(BRIGHTNESS, SATURATION));
  }

  State state;

  cv::namedWindow("Tracks");
  cv::setMouseCallback("Tracks", onMouse, static_cast<void*>(&state));

  cv::Mat image;
  cv::Mat display;
  bool exit = false;
  bool changed = true;
  bool paused = true;
  int t = 0;

  while (!exit) {
    // Load image if required.
    if (changed) {
      std::string file = makeImageFilename(image_format, t);
      ok = readColorImage(file, image);
      CHECK(ok) << "Could not load image";
    }

    display = image.clone();

    // Get features for this frame.
    std::map<int, ScaleSpacePosition> features;
    TrackListTimeIterator<ScaleSpacePosition> frame(tracks, t);
    frame.getPoints(features);

    // Draw rectangle.
    if (state.rect || state.corner) {
      cv::Rect rect;
      if (state.rect) {
        rect = *state.rect;
      } else {
        rect = cv::Rect(*state.corner, state.position);
      }

      cv::rectangle(display, rect, cv::Scalar(0xCC, 0xCC, 0xCC),
          FLAGS_line_thickness);
    }

    // Draw features.
    std::map<int, ScaleSpacePosition>::const_iterator feature;
    for (feature = features.begin(); feature != features.end(); ++feature) {
      int index = feature->first;
      const ScaleSpacePosition& position = feature->second;

      bool outside_rect = false;
      if (state.rect) {
        if (!state.rect->contains(position.point())) {
          outside_rect = true;
        }
      }

      cv::Scalar color = colors[index];
      if (!outside_rect) {
        color *= 0.5;
      }

      ScaleSpaceFeatureDrawer drawer(position, FLAGS_radius);
      drawer.draw(display, color, FLAGS_line_thickness);
    }

    cv::imshow("Tracks", display);
    char c = cv::waitKey(30);

    if (c == 27) {
      exit = true;
    } else if (c == ' ')  {
      if (paused) {
        // Unpause.
        paused = false;
      } else {
        paused = true;
      }
    } else if (c == '\n' || c == '\r')  {
      // If a rectangle is selected, remove all the features outside it.
      if (state.rect) {
        LOG(INFO) << "Do something!";
      }
    } else if (c == 'j') {
      // If we weren't paused, we are now.
      paused = true;

      // Move to next frame.
      t = (t + 1) % num_frames;
      changed = true;
    } else if (c == 'k') {
      // If we weren't paused, we are now.
      paused = true;

      // Move to previous frame.
      t = (t + num_frames - 1) % num_frames;
      changed = true;
    }

    if (!paused) {
      t = (t + 1) % num_frames;
      changed = true;
    }
  }

  return 0;
}
