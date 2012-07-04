#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "track.hpp"
#include "tracker.hpp"
#include "klt_tracker.hpp"

// Number of features.
int MAX_NUM_FEATURES = 2048;
// Threshold on cornerness. Default: 1 (allows many).
int MIN_EIGENVALUE =  100;
// Minimum separation between new features. Default: 10.
int MIN_CLEARANCE = 10;
// Size of window to track. Default: 7.
int WINDOW_SIZE = 7;
// Subsample cornerness to save time. Default: 0.
int CORNERNESS_JUMP = 2;
// Minimum determinant to deem lost. Default: 0.01.
double MIN_DETERMINANT = 0.01;
// Maximum number of iterations. Default: 10.
int MAX_ITERATIONS = 20;
// Minimum step size to deem converged. Default: 0.1.
double MIN_DISPLACEMENT = 0.1;
// Maximum residual to deem lost. Default: 10.0.
double MAX_RESIDUAL = 10.0;

// Visualization settings.
const cv::Scalar MARKER_COLOR(0xFF, 0x00, 0x00);
const int MARKER_RADIUS = 2;
const int MARKER_THICKNESS = 2;
const int TRAIL_LENGTH = 5;
const int TRAIL_THICKNESS = 2;


bool readImage(const std::string& filename, cv::Mat& color, cv::Mat& gray) {
  // Attempt to read next image.
  color = cv::imread(filename, 1);
  if (color.empty()) {
    return false;
  }

  // Convert to gray.
  cvtColor(color, gray, CV_BGR2GRAY);

  return true;
}

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

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " input output" << std::endl;
    return 1;
  }

  // Frame index.
  int n = 0;

  // Read first image to get dimensions.
  cv::Mat color_image;
  cv::Mat image;
  std::string input_format = argv[1];
  std::string input_filename;
  input_filename = boost::str(boost::format(input_format) % (n + 1));

  bool ok = readImage(input_filename, color_image, image);
  if (!ok) {
    std::cerr << "could not read first image" << std::endl;
    return 1;
  }

  // Set up output file.
  std::string output_filename = argv[2];
  cv::FileStorage fs(output_filename, cv::FileStorage::WRITE);

  // Write out image size.
  fs << "width" << image.cols << "height" << image.rows;
  // Open list of tracks.
  fs << "tracks" << "[";

  KltTracker klt_tracker;
  klt_tracker.init(MAX_NUM_FEATURES, MIN_CLEARANCE, WINDOW_SIZE, MIN_EIGENVALUE,
      CORNERNESS_JUMP, MIN_DETERMINANT, MAX_ITERATIONS, MIN_DISPLACEMENT,
      MAX_RESIDUAL);
  SerialTracker& tracker = klt_tracker;

  // Loop variables.
  bool user_quit = false;
  bool have_frame = true;

  while (!user_quit && have_frame) {
    // Attempt to read next frame.
    input_filename = boost::str(boost::format(input_format) % (n + 1));
    have_frame = readImage(input_filename, color_image, image);
    if (!have_frame) {
      continue;
    }

    // Add image to sequence.
    tracker.feed(image);

    // Write out updated tracks.
    tracker.write(fs);

    // Draw the points which are currently being tracked.
    std::vector<const Track*> tracks;
    tracker.activeTracks(tracks);
    drawTracks(color_image, tracks);

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

  fs << "]";

  return 0;
}
