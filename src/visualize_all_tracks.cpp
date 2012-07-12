#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cstdlib>
#include <fstream>
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

typedef std::vector<TrackList> TrackListList;

std::string makeFilename(const std::string& format, int n) {
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
void drawTailRange(cv::Mat& image,
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

void drawTail(cv::Mat& image,
              const TrackCursor& cursor,
              const cv::Scalar& color,
              int start_t) {
  const Track::const_iterator& it = cursor.point;
  int t = it->first;

  // Create either a forward or reverse iterator range.
  if (t < start_t) {
    Track::const_iterator begin(it);
    Track::const_iterator end = cursor.track->end();
    end = findTailEnd(begin, end, t, start_t, MAX_TAIL_LENGTH);
    drawTailRange(image, color, begin, end);
  } else {
    Track::const_reverse_iterator begin(it);
    --begin;
    Track::const_reverse_iterator end = cursor.track->rend();
    end = findTailEnd(begin, end, t, start_t, MAX_TAIL_LENGTH);
    drawTailRange(image, color, begin, end);
  }
}

void drawFeature(cv::Mat& image,
                 const TrackCursor& cursor,
                 const cv::Scalar& color,
                 int start_t) {
  Track::const_iterator it = cursor.point;
  int t = it->first;

  // Draw point.
  cv::circle(image, it->second, POINT_RADIUS, color, -1);

  // Draw tail.
  drawTail(image, cursor, color, start_t);
}

void drawFeatureIfPresent(cv::Mat& image,
                          const ColoredCursor& cursor,
                          int t,
                          int start_t) {
  Track::const_iterator it = cursor.cursor.point;
  int u = it->first;

  if (t == u) {
    // Cursor points to this frame. Draw feature.
    drawFeature(image, cursor.cursor, cursor.color, start_t);
  }
}

void advanceIfEqual(int t, ColoredCursor& cursor) {
  Track::const_iterator& iterator = cursor.cursor.point;
  int u = iterator->first;

  if (t == u) {
    ++iterator;
  }
}

bool fileExists(const std::string& filename) {
  return std::ifstream(filename.c_str()).good();
}

bool loadAllTracks(const std::string& tracks_format,
                   TrackListList& track_lists,
                   int* num_frames) {
  bool ok = true;
  bool have_frame = true;
  int t = 0;

  while (have_frame && ok) {
    // Check if file exists.
    std::string tracks_file = makeFilename(tracks_format, t);
    have_frame = fileExists(tracks_file);
    if (!have_frame) {
      // End of movie.
      continue;
    }

    // Add an empty track list.
    track_lists.push_back(TrackList());

    // Attempt to load tracks.
    cv::Size size;
    ok = loadTracks(tracks_file, size, track_lists.back(), NULL);
    if (!ok) {
      // Failed.
      continue;
    }

    t += 1;
  }

  if (num_frames != NULL) {
    *num_frames = t;
  }

  return true;
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Visualizes features tracked outwards from many frames." << std::endl;
  usage << std::endl;
  usage << argv[0] << " image-format tracks-format" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string image_format = argv[1];
  std::string tracks_format = argv[2];
  std::string output_format = FLAGS_output_format;

  // Load all tracks.
  std::cerr << "loading all tracks..." << std::endl;
  TrackListList track_lists;
  int num_frames;
  bool ok = loadAllTracks(tracks_format, track_lists, &num_frames);
  if (!ok) {
    std::cerr << "could not load tracks" << std::endl;
    return 1;
  }
  std::cerr << "loaded" << std::endl;

  typedef std::list<ColoredCursor> CursorList;
  typedef std::vector<CursorList> CursorListList;

  // Initialize lists of cursors.
  CursorListList cursor_lists;
  for (TrackListList::const_iterator tracks = track_lists.begin();
       tracks != track_lists.end();
       ++tracks) {
    // Add list of cursors.
    cursor_lists.push_back(CursorList());

    // Construct cursors with random colors.
    std::transform(tracks->begin(), tracks->end(),
        std::back_inserter(cursor_lists.back()), ColoredCursor::make);
  }

  // Visualize frames.
  for (int t = 0; t < num_frames; t += 1) {
    std::cerr << (t + 1) << std::endl;

    // Load image from file.
    cv::Mat image;
    cv::Mat gray_image;
    std::string image_file = makeFilename(image_format, t);
    ok = readImage(image_file, image, gray_image);
    if (!ok) {
      throw std::runtime_error("ran out of images when tracks still remain");
    }

    // Frame from which these tracks originated.
    int start_t = 0;

    for (CursorListList::const_iterator cursors = cursor_lists.begin();
         cursors != cursor_lists.end();
         ++cursors) {
      // Draw all features which appear in this frame.
      std::for_each(cursors->begin(), cursors->end(),
          boost::bind(drawFeatureIfPresent, boost::ref(image), _1, t, start_t));

      start_t += 1;
    }

    if (FLAGS_save) {
      std::string output_file = makeFilename(output_format, t);
      cv::imwrite(output_file, image);
    }

    if (FLAGS_display) {
      cv::imshow("tracks", image);
      cv::waitKey(10);
    }

    for (CursorListList::iterator cursors = cursor_lists.begin();
         cursors != cursor_lists.end();
         ++cursors) {
      // Advance cursors.
      std::for_each(cursors->begin(), cursors->end(),
          boost::bind(advanceIfEqual, t, _1));

      // Remove cursors which have ended.
      CursorList::iterator it = cursors->begin();
      while (it != cursors->end()) {
        if (it->cursor.point == it->cursor.track->end()) {
          it = cursors->erase(it);
        } else {
          ++it;
        }
      }
    }
  }

  return 0;
}
