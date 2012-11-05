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

#include "track_list.hpp"
#include "track.hpp"
#include "viterbi.hpp"

#include "read_image.hpp"

DEFINE_int32(width, 1280, "Screen width");
DEFINE_int32(height, 800, "Screen width");

std::string makeFrameFilename(const std::string& format, int time) {
  return boost::str(boost::format(format) % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Linear time offline tracking" << std::endl;
  usage << std::endl;
  usage << argv[0] << " image-format num-frames" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

bool tooBig(const cv::Size& image, const cv::Size& screen) {
  return image.width > screen.width || image.height > screen.height;
}

void loadImages(const std::string& format,
                int num_frames,
                const cv::Size& screen,
                std::vector<cv::Mat>& images) {
  images.clear();
  int downsample = 0;

  for (int t = 0; t < num_frames; t += 1) {
    cv::Mat image;
    std::string file = makeFrameFilename(format, t);
    bool ok = readColorImage(file, image);
    CHECK(ok) << "Could not load image";

    if (t == 0) {
      while (tooBig(image.size(), screen)) {
        cv::pyrDown(image, image);
        downsample += 1;
      }
    } else {
      for (int i = 0; i < downsample; i += 1) {
        cv::pyrDown(image, image);
      }
    }

    images.push_back(image);
  }
}

struct Parameters {
  cv::Size size;
};

struct State {
  bool paused;
  int time;
  const std::vector<cv::Mat>* images;
};

struct Data {
  State* state;
  const Parameters* parameters;
};

void onMouse(int event, int x, int y, int, void* tag) {
  Data* data = static_cast<Data*>(tag);
  const Parameters& parameters = *data->parameters;
  State& state = *data->state;

  if (state.paused) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      // Track the point.

      // Extract a patch for cross-correlation.
      int x_radius = (parameters.size.width - 1) / 2;
      int y_radius = (parameters.size.height - 1) / 2;
      cv::Point offset(x - x_radius, y - y_radius);
      cv::Rect region(offset, parameters.size);

      cv::Mat original = (*state.images)[state.time](region);

      //linearTimeOfflineTracking(original, state.images, 1);

      LOG(INFO) << "Clicked (" << x << ", " << y << ", " << state.time << ")";
    }
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  int num_frames = boost::lexical_cast<int>(argv[2]);
  CHECK(num_frames > 0) << "Need at least one frame, two would be great";
  cv::Size screen_size(FLAGS_width, FLAGS_height);

  // Pre-load images.
  LOG(INFO) << "Loading images...";
  std::vector<cv::Mat> color_images;
  loadImages(image_format, num_frames, screen_size, color_images);

  // Convert to grayscale.
  LOG(INFO) << "Loaded " << color_images.size() << " images";
  std::vector<cv::Mat> gray_images;
  std::vector<cv::Mat>::const_iterator color_image;
  for (color_image = color_images.begin();
       color_image != color_images.end();
       ++color_image) {
    cv::Mat gray_image;
    cv::cvtColor(*color_image, gray_image, CV_BGR2GRAY);
    gray_images.push_back(gray_image);
  }

  TrackList<cv::Point2d> track;

  bool exit = false;
  State state;
  state.time = 0;
  state.paused = false;
  state.images = &gray_images;

  Parameters parameters;
  parameters.size = cv::Size(11, 11);

  Data data;
  data.state = &state;
  data.parameters = &parameters;

  cv::namedWindow("video");
  cv::setMouseCallback("video", onMouse, &data);

  while (!exit) {
    char c = cv::waitKey(30);

    if (c == 27) {
      exit = true;
    } else {
      if (c == ' ') {
        state.paused = !state.paused;
      }

      if (!state.paused) {
        // Show next frame.
        state.time = (state.time + 1) % num_frames;
        cv::imshow("video", color_images[state.time]);
      } else {
        if (c == 'j') {
          // Show next frame.
          state.time = (state.time + 1) % num_frames;
          cv::imshow("video", color_images[state.time]);
        } else if (c == 'k') {
          // Show previous frame.
          state.time = (state.time + num_frames - 1) % num_frames;
          cv::imshow("video", color_images[state.time]);
        }
      }
    }
  }

  return 0;
}
