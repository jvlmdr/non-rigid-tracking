#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "read_image.hpp"
#include "track.hpp"
#include "tracker.hpp"
#include "klt_tracker.hpp"

#include "image_point_writer.hpp"
#include "track_list_writer.hpp"

// Size of window to track. Default: 7.
const int WINDOW_SIZE = 7;
// Minimum determinant to deem lost. Default: 0.01.
const double MIN_DETERMINANT = 0.01;
// Maximum number of iterations. Default: 10.
const int MAX_ITERATIONS = 20;
// Minimum step size to deem converged. Default: 0.1.
const double MIN_DISPLACEMENT = 0.1;
// Maximum residual to deem lost. Default: 10.0.
const double MAX_RESIDUAL = 10.0;
// Consistency checking mode. Default: -1.
// -1: none, 0: translation, 1: similarity, 2: affine
const int CONSISTENCY_MODE = 2;

// Visualization settings.
const cv::Scalar MARKER_COLOR(0xFF, 0x00, 0x00);
const int MARKER_RADIUS = 2;
const int MARKER_THICKNESS = 2;
const int TRAIL_LENGTH = 5;
const int TRAIL_THICKNESS = 2;

typedef Track_<cv::Point2d> Track;
typedef TrackList_<cv::Point2d> TrackList;

void drawTrack(cv::Mat& image, const Track& track) {
  // Get last point in track.
  const cv::Point2d& pos = track.rbegin()->second;
  // Draw circle around point.
  cv::circle(image, pos, MARKER_RADIUS, MARKER_COLOR, MARKER_THICKNESS);

  // Draw trail from previous frames.
  Track::const_reverse_iterator point = track.rbegin();
  cv::Point2d prev = point->second;
  ++point;
  int n = 0;

  while (n < TRAIL_LENGTH && point != track.rend()) {
    cv::line(image, prev, point->second, MARKER_COLOR, TRAIL_THICKNESS);
    prev = point->second;
    ++point;
    n += 1;
  }
}

void drawTracks(cv::Mat& image, const std::vector<const Track*>& tracks) {
  typedef std::vector<const Track*> TrackList;
  TrackList::const_iterator track;

  for (track = tracks.begin(); track != tracks.end(); ++track) {
    drawTrack(image, **track);
  }
}

std::string imageFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

cv::Point2d readPointFromFile(const cv::FileNode& node) {
  double x = (double)node["x"];
  double y = (double)node["y"];

  return cv::Point2d(x, y);
}

void shiftTrack(const Track& input, Track& output, int delta) {
  output.clear();

  Track::const_iterator point;
  for (point = input.begin(); point != input.end(); ++point) {
    int t = point->first;
    const cv::Point2d& position = point->second;

    output[t + delta] = position;
  }
}

void shiftTracks(const TrackList& input, TrackList& output, int delta) {
  output.clear();

  TrackList::const_iterator track;
  for (track = input.begin(); track != input.end(); ++track) {
    // Create a new track.
    output.push_back(Track());
    // Copy with a shift.
    shiftTrack(*track, output.back(), delta);
  }
}

bool loadKeypoints(const std::string& filename,
                   std::vector<cv::Point2d>& points) {
  // Open file.
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }

  // Parse keypoints.
  cv::FileNode list = fs["keypoints"];
  std::transform(list.begin(), list.end(), std::back_inserter(points),
      readPointFromFile);

  return true;
}

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "usage: " << argv[0] <<
      " image-format frame-number keypoints tracks" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Example" << std::endl;
    std::cerr << argv[0] <<
      " input/my-video/%03d.png 5 output/my-video/keypoints/005.yaml"
      " output/my-video/tracks/005.yaml" << std::endl;
    return 1;
  }

  std::string image_format = argv[1];
  int first_frame = boost::lexical_cast<int>(argv[2]);
  std::string keypoints_filename = argv[3];
  std::string tracks_filename = argv[4];

  // Frame index.
  int n = first_frame;

  // Read keypoints from file.
  std::vector<cv::Point2d> points;
  loadKeypoints(keypoints_filename, points);
  std::cerr << points.size() << std::endl;

  KltTracker klt_tracker;
  klt_tracker.init(points, WINDOW_SIZE, MIN_DETERMINANT, MAX_ITERATIONS,
      MIN_DISPLACEMENT, MAX_RESIDUAL, CONSISTENCY_MODE);
  // Ensure that we're adhering to the tracker interface.
  SerialTracker& tracker = klt_tracker;

  // Loop variables.
  bool user_quit = false;
  bool have_frame = true;
  bool have_features = true;
  cv::Mat color_image;
  cv::Mat image;
  cv::Size size(0, 0);

  while (!user_quit && have_frame && have_features) {
    // Attempt to read next frame.
    have_frame = readImage(imageFilename(image_format, n), color_image, image);
    if (!have_frame) {
      continue;
    }

    // Get image size to save out to file.
    size = image.size();

    // Add image to sequence.
    tracker.feed(image);

    // Draw the points which are currently being tracked.
    std::vector<const Track*> tracks;
    tracker.activeTracks(tracks);
    drawTracks(color_image, tracks);

    // If there are no features, then abort.
    have_features = !tracks.empty();

    // Draw image.
    cv::imshow("tracking", color_image);

    // Check for keypress.
    char c = char(cv::waitKey(10));
    if (c == 27) {
      user_quit = true;
    }

    // Frame counter.
    n += 1;
  }

  // Shift tracks in time so that they start at the first frame.
  TrackList shifted;
  shiftTracks(tracker.tracks(), shifted, first_frame);

  // Write out tracks.
  ImagePointWriter writer;
  bool ok = saveTrackList(tracks_filename, shifted, writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
