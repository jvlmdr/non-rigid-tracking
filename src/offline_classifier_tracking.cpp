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
#include "image_file_sequence.hpp"
#include "dynamic_program_occlusion_tracker.hpp"

#include "track_list_writer.hpp"
#include "image_point_writer.hpp"

DEFINE_int32(radius, 5, "Radius of patch");
DEFINE_double(lambda, 1., "Temporal regularization");
DEFINE_double(rho, 1., "ADMM parameter");
DEFINE_double(penalty, 0.5, "The cost of occlusion");

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

struct State {
  bool* paused;
  const int* time;
  const Video* video;
  int radius;
  TrackList<cv::Point2d>* tracks;
  TrackList<cv::Point>* seeds;
  OfflineTracker* tracker;
};

/*
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

//  // Convert to a track.
//  track.clear();
//  int num_frames = images.size();
//
//  for (int t = 0; t < num_frames; t += 1) {
//    int rx = (templ.cols - 1) / 2 + 1;
//    int ry = (templ.rows - 1) / 2 + 1;
//
//    track[t] = cv::Point2d(x[t][1] + rx, x[t][0] + ry);
//  }
}
*/

void onMouse(int event, int x, int y, int, void* tag) {
  State& state = *static_cast<State*>(tag);

  if (*state.paused) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      // Extract a patch for cross-correlation.
      cv::Point seed(x, y);

      // Track the point.
      Track<cv::Point2d> track;

      SpaceTimeImagePoint point(seed, *state.time);
      state.tracker->track(point, track);
      //admmOfflineTracking(templ, *parameters.images, FLAGS_lambda, FLAGS_rho,
      //    track);
      //linearTimeOfflineTracking(templ, *parameters.images, FLAGS_lambda, track);
      //findBestInEveryFrame(templ, *parameters.images, track);

      state.tracks->push_back(Track<cv::Point2d>());
      state.tracks->back().swap(track);

      state.seeds->push_back(Track<cv::Point>());
      state.seeds->back()[*state.time] = cv::Point(x, y);

      *state.paused = false;
    }
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  int num_frames = boost::lexical_cast<int>(argv[2]);
  std::string points_file = argv[3];
  std::string tracks_file = argv[4];

  CHECK(num_frames > 0) << "Need at least one frame, two would probably be "
      "more interesting";

  int radius = FLAGS_radius;

  // Set up video stream.
  ImageFileSequence video(boost::format(image_format), num_frames, true);

  // Set up tracker.
  DynamicProgramOcclusionTracker tracker(FLAGS_lambda, FLAGS_penalty,
      FLAGS_radius);
  tracker.init(video);

  bool exit = false;
  int time = 0;
  bool paused = true;
  TrackList<cv::Point2d> tracks;
  TrackList<cv::Point> seeds;

  State state;
  state.time = &time;
  state.paused = &paused;
  state.video = &video;
  state.radius = radius;
  state.tracks = &tracks;
  state.seeds = &seeds;
  state.tracker = &tracker;

  cv::namedWindow("video");
  cv::setMouseCallback("video", onMouse, &state);

  while (!exit) {
    char c = cv::waitKey(30);

    if (c == 27) {
      exit = true;
    } else {
      if (c == ' ') {
        paused = !paused;
      }

      if (!paused) {
        // Show next frame.
        time = (time + 1) % num_frames;
      } else {
        if (c == 'j') {
          // Show next frame.
          time = (time + 1) % num_frames;
        } else if (c == 'k') {
          // Show previous frame.
          time = (time + num_frames - 1) % num_frames;
        }
      }

      // Load image for this instant.
      cv::Mat image;
      video.get(time, image);

      // Convert grayscale back to color for displaying.
      cv::Mat display;
      cv::cvtColor(image, display, CV_GRAY2BGR);

      // Draw tracks on frame.
      std::map<int, cv::Point2d> keypoints;
      TrackListTimeIterator<cv::Point2d> frame(tracks, time);
      frame.getPoints(keypoints);

      std::map<int, cv::Point2d>::const_iterator iter;
      for (iter = keypoints.begin(); iter != keypoints.end(); ++iter) {
        const cv::Point2d& keypoint = iter->second;

        cv::Point2d pt1 = keypoint - cv::Point2d(radius, radius);
        cv::Point2d pt2 = keypoint + cv::Point2d(radius, radius);
        cv::rectangle(display, pt1, pt2, cv::Scalar(0, 0, 255), 1);
      }

      cv::imshow("video", display);
    }
  }

  bool ok;

  // Save points and tracks out.
  ImagePointWriter<int> pixel_writer;
  ok = saveTrackList(points_file, seeds, pixel_writer);
  CHECK(ok) << "Could not save points";

  ImagePointWriter<double> point_writer;
  ok = saveTrackList(tracks_file, tracks, point_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
