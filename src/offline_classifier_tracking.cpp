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
#include "admm_tracking.hpp"

#include "read_image.hpp"

#include "track_list_writer.hpp"
#include "image_point_writer.hpp"

DEFINE_int32(width, 1280, "Screen width");
DEFINE_int32(height, 800, "Screen width");

DEFINE_double(lambda, 1., "Temporal regularization");
DEFINE_double(rho, 1., "ADMM parameter");

const int RADIUS = 5;
const int DIAMETER = 2 * RADIUS + 1;

std::string makeFrameFilename(const std::string& format, int time) {
  return boost::str(boost::format(format) % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Linear time offline tracking" << std::endl;
  usage << std::endl;
  usage << argv[0] << " image-format num-frames points tracks" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 5) {
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
  const std::vector<cv::Mat>* images;
  cv::Size size;
};

struct State {
  bool paused;
  int time;
  TrackList<cv::Point2d> tracks;
  TrackList<cv::Point> seeds;
  //std::vector<cv::Scalar> colors;
};

struct Data {
  State* state;
  const Parameters* parameters;
};

void findBestInEveryFrame(const cv::Mat& templ,
                          const std::vector<cv::Mat>& images,
                          Track<cv::Point2d>& track) {
  track.clear();

  LOG(INFO) << "Performing cross-correlation...";

  std::vector<cv::Mat>::const_iterator image;
  int t = 0;

  for (image = images.begin(); image != images.end(); ++image) {
    // Evaluate response to template.
    cv::Mat response;
    cv::matchTemplate(*image, templ, response, cv::TM_CCORR_NORMED);

    // Convert to 64-bit.
    response = cv::Mat_<double>(response);
    response = -1e3 * response;

    // Find min in response.
    cv::Point arg;
    cv::minMaxLoc(response, NULL, NULL, &arg, NULL);

    int rx = (templ.cols - 1) / 2 + 1;
    int ry = (templ.rows - 1) / 2 + 1;
    arg.x += rx;
    arg.y += ry;

    track[t] = arg;

    t += 1;
  }
}

void admmOfflineTracking(const cv::Mat& templ,
                         const std::vector<cv::Mat>& images,
                         double lambda,
                         double rho,
                         Track<cv::Point2d>& track) {
  // Match the template to every image.
  std::vector<cv::Mat> responses;

  LOG(INFO) << "Performing cross-correlation...";
  std::vector<cv::Mat>::const_iterator image;
  for (image = images.begin(); image != images.end(); ++image) {
    // Evaluate response to template.
    cv::Mat response;
    cv::matchTemplate(*image, templ, response, cv::TM_CCORR_NORMED);
    // Convert to 64-bit.
    response = cv::Mat_<double>(response);
    response = -1. * response;
    // Add to list.
    responses.push_back(response);
  }

  LOG(INFO) << "Solving ADMM...";
  std::vector<cv::Point2d> x;
  findClassifierTrackAdmm(responses, x, lambda, rho);

  /*
  // Convert to a track.
  track.clear();
  int num_frames = images.size();

  for (int t = 0; t < num_frames; t += 1) {
    int rx = (templ.cols - 1) / 2 + 1;
    int ry = (templ.rows - 1) / 2 + 1;

    track[t] = cv::Point2d(x[t][1] + rx, x[t][0] + ry);
  }
  */
}

void linearTimeOfflineTracking(const cv::Mat& templ,
                               const std::vector<cv::Mat>& images,
                               double lambda,
                               Track<cv::Point2d>& track) {
  // Match the template to every image.
  std::vector<cv::Mat> responses;
  int n = images.size();

  LOG(INFO) << "Performing cross-correlation...";
  std::vector<cv::Mat>::const_iterator image;
  for (image = images.begin(); image != images.end(); ++image) {
    // Evaluate response to template.
    cv::Mat response;
    cv::matchTemplate(*image, templ, response, cv::TM_CCORR_NORMED);
    // Convert to 64-bit.
    response = cv::Mat_<double>(response);
    response = -1. / (lambda / n) * response;
    // Add to list.
    responses.push_back(response);
  }

  LOG(INFO) << "Solving dynamic program...";
  std::vector<cv::Vec2i> x;
  solveViterbiQuadratic2D(responses, x);

  // Convert to a track.
  track.clear();
  int num_frames = images.size();

  for (int t = 0; t < num_frames; t += 1) {
    int rx = (templ.cols - 1) / 2 + 1;
    int ry = (templ.rows - 1) / 2 + 1;

    track[t] = cv::Point2d(x[t][1] + rx, x[t][0] + ry);
  }
}

void onMouse(int event, int x, int y, int, void* tag) {
  Data* data = static_cast<Data*>(tag);
  const Parameters& parameters = *data->parameters;
  State& state = *data->state;

  if (state.paused) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      // Extract a patch for cross-correlation.
      int x_radius = (parameters.size.width - 1) / 2;
      int y_radius = (parameters.size.height - 1) / 2;
      cv::Point offset(x - x_radius - 1, y - y_radius - 1);
      cv::Rect region(offset, parameters.size);
      cv::Mat original = (*parameters.images)[state.time](region);
      cv::imshow("template", original);

      // Track the point.
      Track<cv::Point2d> track;
      //admmOfflineTracking(original, *parameters.images, FLAGS_lambda, FLAGS_rho,
      //    track);
      linearTimeOfflineTracking(original, *parameters.images, FLAGS_lambda,
          track);
      //findBestInEveryFrame(original, *parameters.images, track);

      state.tracks.push_back(Track<cv::Point2d>());
      state.tracks.back().swap(track);

      state.seeds.push_back(Track<cv::Point>());
      state.seeds.back()[state.time] = cv::Point(x, y);

      state.paused = false;
    }
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  int num_frames = boost::lexical_cast<int>(argv[2]);
  std::string points_file = argv[3];
  std::string tracks_file = argv[4];

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
  state.paused = true;

  Parameters parameters;
  parameters.size = cv::Size(DIAMETER, DIAMETER);
  parameters.images = &gray_images;

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
      } else {
        if (c == 'j') {
          // Show next frame.
          state.time = (state.time + 1) % num_frames;
        } else if (c == 'k') {
          // Show previous frame.
          state.time = (state.time + num_frames - 1) % num_frames;
        }
      }

      cv::Mat display = color_images[state.time].clone();

      // Draw tracks on frame.
      std::map<int, cv::Point2d> keypoints;
      TrackListTimeIterator<cv::Point2d> frame(state.tracks, state.time);
      frame.getPoints(keypoints);

      std::map<int, cv::Point2d>::const_iterator iter;
      for (iter = keypoints.begin(); iter != keypoints.end(); ++iter) {
        const cv::Point2d& keypoint = iter->second;

        cv::Point2d pt1 = keypoint - cv::Point2d(RADIUS + 1, RADIUS + 1);
        cv::Point2d pt2 = keypoint + cv::Point2d(RADIUS + 1, RADIUS + 1);
        cv::rectangle(display, pt1, pt2, cv::Scalar(0, 0, 255), 2);
      }

      cv::imshow("video", display);
    }
  }

  bool ok;

  // Save points and tracks out.
  ImagePointWriter<int> pixel_writer;
  ok = saveTrackList(points_file, state.seeds, pixel_writer);
  CHECK(ok) << "Could not save points";

  ImagePointWriter<double> point_writer;
  ok = saveTrackList(tracks_file, state.tracks, point_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
