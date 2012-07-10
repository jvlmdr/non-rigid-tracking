#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cstdlib>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <gflags/gflags.h>
#include "read_image.hpp"
#include "track.hpp"
#include "track_list.hpp"
#include "random_color.hpp"

const int MAX_TAIL_LENGTH = 10;
const int POINT_RADIUS = 3;
const int TAIL_THICKNESS = 1;
const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

DEFINE_string(output_format, "%d.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

std::string imageFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

struct ColoredCursor {
  TrackCursor cursor;
  cv::Scalar color;

  ColoredCursor(const TrackCursor& cursor, const cv::Scalar& color)
      : cursor(cursor), color(color) {}

  ColoredCursor() : cursor(), color() {}

  static ColoredCursor make(const Track& track) {
    return ColoredCursor(TrackCursor::make(track),
        randomColor(SATURATION, BRIGHTNESS));
  }
};

template<class TrackPointIterator>
TrackPointIterator findTailEnd(TrackPointIterator begin,
                               TrackPointIterator end,
                               const int t,
                               const int start_t,
                               const int max_length) {
  // Check if current frame is before or after start frame.
  int n = 0;

  // Loop variables.
  TrackPointIterator it = begin;
  bool found = false;

  while (it != end && !found) {
    // Get timestamp of this frame.
    int u = it->first;

    if (std::abs(t - u) >= max_length) {
      // Reached maximum tail length.
      found = true;
    } else {
      bool passed_start;

      if (t < start_t) {
        passed_start = (u > start_t);
      } else if (t > start_t) {
        passed_start = (u < start_t);
      } else {
        passed_start = (u != start_t);
      }

      if (passed_start) {
        found = true;
      }
    }

    if (!found) {
      ++it;
      n += 1;
    }
  }

  return it;
}

template<class TrackPointIterator>
void drawTail(cv::Mat& image,
              const cv::Scalar& color,
              TrackPointIterator begin,
              TrackPointIterator end) {
  TrackPointIterator it = begin;
  bool first = true;
  cv::Point2d previous;

  for (it = begin; it != end; ++it) {
    const cv::Point2d& current = it->second;

    // Can't draw anything in the first frame.
    if (!first) {
      cv::line(image, previous, current, color, TAIL_THICKNESS);
    }

    previous = current;
    first = false;
  }
}

void drawPoint(cv::Mat& image,
               int t,
               int start_t,
               const ColoredCursor& cursor) {
  Track::const_iterator it = cursor.cursor.point;
  int u = it->first;

  if (t == u) {
    // Cursor is at this point. Draw point.
    const cv::Point2d& position = it->second;
    cv::circle(image, position, POINT_RADIUS, cursor.color, -1);

    if (t < start_t) {
      Track::const_iterator begin(it);
      Track::const_iterator end = cursor.cursor.track->end();
      end = findTailEnd(begin, end, t, start_t, MAX_TAIL_LENGTH);
      drawTail(image, cursor.color, begin, end);
    } else {
      Track::const_reverse_iterator begin(it);
      --begin;
      Track::const_reverse_iterator end = cursor.cursor.track->rend();
      end = findTailEnd(begin, end, t, start_t, MAX_TAIL_LENGTH);
      drawTail(image, cursor.color, begin, end);
    }
  }
}

void advanceIfEqual(int t, ColoredCursor& cursor) {
  Track::const_iterator& iterator = cursor.cursor.point;
  int u = iterator->first;

  if (t == u) {
    ++iterator;
  }
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Visualizes tracks outwards from a given frame." << std::endl;
  usage << std::endl;
  usage << argv[0] << " image-format tracks frame-number" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string image_format = argv[1];
  std::string tracks_file = argv[2];
  int initial_frame = boost::lexical_cast<int>(argv[3]);
  std::string output_format = FLAGS_output_format;

  bool ok;

  // Load tracks.
  cv::Size size;
  TrackList tracks;
  ok = loadTracks(tracks_file, size, tracks);

  typedef std::list<ColoredCursor> CursorList;
  CursorList cursors(tracks.size());
  std::transform(tracks.begin(), tracks.end(), cursors.begin(),
      ColoredCursor::make);

  // Find the index of the first frame.
  int t = findFirstFrame(tracks);

  // Visualize frames until we have no tracks left to draw.
  while (!cursors.empty()) {
    // Load image from file.
    cv::Mat image;
    cv::Mat gray_image;
    std::string image_file = imageFilename(image_format, t);
    ok = readImage(image_file, image, gray_image);
    if (!ok) {
      throw std::runtime_error("could not read image");
    }

    // Draw the points that appear in this frame.
    std::for_each(cursors.begin(), cursors.end(),
        boost::bind(drawPoint, boost::ref(image), t, initial_frame, _1));

    if (FLAGS_display) {
      cv::imshow("tracks", image);
      cv::waitKey(10);
    }

    if (FLAGS_save) {
      std::string output_file = imageFilename(output_format, t);
      cv::imwrite(output_file, image);
    }

    // Advance cursors.
    std::for_each(cursors.begin(), cursors.end(),
        boost::bind(advanceIfEqual, t, _1));

    // Remove cursors which have ended.
    CursorList::iterator it = cursors.begin();
    while (it != cursors.end()) {
      if (it->cursor.point == it->cursor.track->end()) {
        it = cursors.erase(it);
      } else {
        ++it;
      }
    }

    t += 1;
  }

  /*
  // Load images.
  cv::Mat image1;
  cv::Mat image2;
  cv::Mat gray;
  ok = readImage(image_file1, image1, gray);
  if (!ok) {
    std::cerr << "could not load image" << std::endl;
    return 1;
  }
  ok = readImage(image_file2, image2, gray);
  if (!ok) {
    std::cerr << "could not load image" << std::endl;
    return 1;
  }

  // Load keypoints.
  KeyPointList keypoints1;
  KeyPointList keypoints2;
  ok = loadKeyPoints(keypoints_file1, keypoints1);
  if (!ok) {
    std::cerr << "could not load keypoints" << std::endl;
    return 1;
  }
  ok = loadKeyPoints(keypoints_file2, keypoints2);
  if (!ok) {
    std::cerr << "could not load keypoints" << std::endl;
    return 1;
  }

  // Visualize matches.
  cv::Mat render;
  drawMatches(keypoints1, keypoints2, matches, image1, image2, render);

  if (FLAGS_save) {
    cv::imwrite(output_file, render);
  }
  if (FLAGS_display) {
    cv::imshow("matches", render);
    cv::waitKey(0);
  }
  */

  return 0;
}
