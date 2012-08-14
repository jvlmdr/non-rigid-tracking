#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <limits>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "read_image.hpp"
#include "track.hpp"
#include "track_list.hpp"
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

const int TIME_WINDOW = 8;
const double MIN_AVERAGE_STEP = 1.0;

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

void shiftTrack(const Track& input, Track& output, int delta, bool reverse) {
  Track::const_iterator point;
  for (point = input.begin(); point != input.end(); ++point) {
    int t = point->first;
    const cv::Point2d& position = point->second;

    if (reverse) {
      t = -t;
    }

    output[t + delta] = position;
  }
}

void shiftTracks(const TrackList& input,
                 TrackList& output,
                 int delta) {
  TrackList::const_iterator track;
  for (track = input.begin(); track != input.end(); ++track) {
    // Create a new track.
    output.push_back(Track());
    // Copy with a shift.
    shiftTrack(*track, output.back(), delta, false);
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

bool startsBefore(const Track& lhs, const Track& rhs) {
  return lhs.begin()->first < rhs.begin()->first;
}

void fuseBidirectional(const TrackList& forward,
                       const TrackList& backward,
                       TrackList& fused,
                       int delta) {
  TrackList::const_iterator forward_track = forward.begin();
  TrackList::const_iterator backward_track = backward.begin();

  while (forward_track != forward.end()) {
    // Create a new track.
    fused.push_back(Track());

    // Copy with a shift.
    shiftTrack(*forward_track, fused.back(), delta, false);
    shiftTrack(*backward_track, fused.back(), delta, true);

    ++forward_track;
    ++backward_track;
  }
}

// Returns number of frames tracked, including first.
int track(const std::string& image_format,
          const std::vector<cv::Point2d>& points,
          TrackList& tracks,
          int offset,
          bool reverse,
          int max_frames) {
  // Non-positive means infinite.
  if (max_frames <= 0) {
    max_frames = std::numeric_limits<int>::max();
  }

  // Initialize the tracker.
  KltTracker tracker;
  tracker.init(points, WINDOW_SIZE, MIN_DETERMINANT, MAX_ITERATIONS,
      MIN_DISPLACEMENT, MAX_RESIDUAL, CONSISTENCY_MODE);

  // Loop variables.
  cv::Mat color_image;
  cv::Mat image;
  int n = 0;
  int i = offset;
  bool have_frame = true;
  bool have_features = true;

  while (have_frame && have_features && n < max_frames) {
    // Attempt to read next frame.
    have_frame = readImage(imageFilename(image_format, i), color_image, image);
    if (!have_frame) {
      continue;
    }

    // Add image to sequence.
    tracker.feed(image);

    // Draw the points which are currently being tracked.
    std::vector<const Track*> tracks;
    tracker.activeTracks(tracks);
    drawTracks(color_image, tracks);

    // Draw image.
    //cv::imshow("tracking", color_image);
    //cv::waitKey(1);

    // If there are no features, then abort.
    have_features = !tracks.empty();
    // Frame index.
    if (reverse) {
      i -= 1;
    } else {
      i += 1;
    }
    n += 1;
  }

  // Copy tracks out.
  std::copy(tracker.tracks().begin(), tracker.tracks().end(),
      std::back_inserter(tracks));

  return n;
}

////////////////////////////////////////////////////////////////////////////////

typedef std::map<int, Track*> TrackSubset;

bool tooShort(const TrackSubset::value_type& x) {
  const Track* track = x.second;
  return (track->size() < TIME_WINDOW * 2 - 1);
}

double maxStep(const Track& track) {
  double max = 0.;
  cv::Point2d previous;
  bool first_point = true;

  Track::const_iterator point;
  for (point = track.begin(); point != track.end(); ++point) {
    const cv::Point2d& current = point->second;

    if (!first_point) {
      cv::Point2d delta = current - previous;
      max = std::max(max, cv::norm(delta));
    }

    first_point = false;
    previous = current;
  }

  return max;
}

double meanStep(const Track& track) {
  double mean = 0.;
  cv::Point2d previous;
  bool first_point = true;

  Track::const_iterator point;
  for (point = track.begin(); point != track.end(); ++point) {
    const cv::Point2d& current = point->second;

    if (!first_point) {
      cv::Point2d delta = current - previous;
      mean += cv::norm(delta);
    }

    first_point = false;
    previous = current;
  }

  mean /= track.size() - 1;

  return mean;
}

bool tooStationary(const TrackSubset::value_type& x) {
  const Track* track = x.second;
  return (meanStep(*track) < MIN_AVERAGE_STEP);
}

////////////////////////////////////////////////////////////////////////////////

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

  // Read image size from first frame.
  cv::Size size;
  {
    cv::Mat image;
    cv::Mat color_image;
    std::string filename = imageFilename(image_format, first_frame);

    bool ok = readImage(filename, color_image, image);
    if (!ok) {
      std::cerr << "could not read first frame" << std::endl;
      return 1;
    }

    size = image.size();
  }

  // Read keypoints from file.
  std::vector<cv::Point2d> points;
  loadKeypoints(keypoints_filename, points);

  // Track forwards.
  TrackList forward;
  std::cerr << "track forwards" << std::endl;
  track(image_format, points, forward, first_frame, false, TIME_WINDOW);

  // Track backwards.
  TrackList backward;
  std::cerr << "track backwards" << std::endl;
  track(image_format, points, backward, first_frame, true, TIME_WINDOW);

  // Merge tracks.
  TrackList tracks;
  fuseBidirectional(forward, backward, tracks, 0);

  // Populate subset.
  TrackSubset track_subset;
  for (int i = 0; i < tracks.size(); ++i) {
    track_subset[i] = &tracks[i];
  }

  // Remove tracks which could not be tracked for the whole window.
  std::cerr << "removing tracks which are too short..." << std::endl;
  TrackSubset long_tracks;
  std::remove_copy_if(track_subset.begin(), track_subset.end(),
      std::inserter(long_tracks, long_tracks.begin()), tooShort);
  track_subset.swap(long_tracks);
  std::cerr << track_subset.size() << " tracks remain" << std::endl;

  // Remove tracks which did not move.
  std::cerr << "removing tracks which did not move..." << std::endl;
  TrackSubset moving_tracks;
  std::remove_copy_if(track_subset.begin(), track_subset.end(),
      std::inserter(moving_tracks, moving_tracks.begin()), tooStationary);
  track_subset.swap(moving_tracks);
  std::cerr << track_subset.size() << " tracks remain" << std::endl;

  // Take subset of points.
  {
    std::vector<cv::Point2d> point_subset;

    TrackSubset::const_iterator track;
    for (track = track_subset.begin(); track != track_subset.end(); ++track) {
      int index = track->first;
      point_subset.push_back(points[index]);
    }

    points.swap(point_subset);
  }

  // Track forwards.
  forward.clear();
  std::cerr << "track forwards" << std::endl;
  track(image_format, points, forward, first_frame, false, -1);

  // Track backwards.
  backward.clear();
  std::cerr << "track backwards" << std::endl;
  track(image_format, points, backward, first_frame, true, -1);

  // Merge tracks.
  tracks.clear();
  fuseBidirectional(forward, backward, tracks, first_frame);

  // Ensure ordering by appearance is maintained.
  std::sort(tracks.begin(), tracks.end(), startsBefore);

  // Write out tracks.
  ImagePointWriter writer;
  bool ok = saveTrackList(tracks_filename, tracks, writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
