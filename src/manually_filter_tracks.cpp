#include <string>
#include <list>
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

#include "track_list_writer.hpp"
#include "scale_space_position_writer.hpp"

DEFINE_int32(radius, 5, "Base feature radius");
DEFINE_int32(line_thickness, 2, "Line thickness for drawing");

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

std::string makeImageFilename(const std::string& format, int time) {
  return boost::str(boost::format(format) % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Allows the user to clean up tracks" << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks image-format num-frames clean-tracks indices" <<
      std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];
  std::string output_file = argv[4];
  std::string indices_file = argv[5];

  bool ok;

  // Load tracks.
  // TODO: Feature needs to have draw(), x and y position and write().
  TrackList<ScaleSpacePosition> track_list;
  ScaleSpacePositionReader feature_reader;
  ok = loadTrackList(tracks_file, track_list, feature_reader);
  CHECK(ok) << "Could not load tracks";

  std::list<Track<ScaleSpacePosition> > tracks;
  std::copy(track_list.begin(), track_list.end(), std::back_inserter(tracks));

  int num_tracks = tracks.size();
  LOG(INFO) << "Loaded " << num_tracks << " tracks";

  // Generate a color for each track.
  std::vector<cv::Scalar> colors;
  for (int i = 0; i < num_tracks; i += 1) {
    colors.push_back(randomColor(BRIGHTNESS, SATURATION));
  }

  cv::namedWindow("Tracks");

  cv::Mat image;
  cv::Mat display;
  bool exit = false;
  bool paused = true;

  std::list<Track<ScaleSpacePosition> >::iterator track = tracks.begin();
  std::vector<cv::Scalar>::iterator color = colors.begin();
  Track<ScaleSpacePosition>::iterator element = track->begin();
  bool changed = true;

  bool marker_set = false;
  Track<ScaleSpacePosition>::iterator marker_element;
  int marker_frame = 0;

  while (!exit) {
    int t = element->first;
    const ScaleSpacePosition& position = element->second;

    // Load image if required.
    if (changed) {
      std::string file = makeImageFilename(image_format, t);
      ok = readColorImage(file, image);
      CHECK(ok) << "Could not load image";
      changed = false;
    }

    // Clone image.
    display = image.clone();

    // Draw feature.
    ScaleSpaceFeatureDrawer drawer(position, FLAGS_radius);
    drawer.draw(display, *color, FLAGS_line_thickness);

    cv::imshow("Tracks", display);
    char c = cv::waitKey(30);

    if (c == 27) {
      exit = true;
    } else if (c == ' ')  {
      if (paused) {
        paused = false;
      } else {
        paused = true;
      }
    } else if (c == 'n') {
      paused = true;

      ++track;
      if (track == tracks.end()) {
        track = tracks.begin();
      }
      element = track->begin();
      marker_set = false;
      changed = true;
    } else if (c == 'N') {
      paused = true;

      if (track == tracks.begin()) {
        track = tracks.end();
      }
      --track;
      element = track->begin();
      marker_set = false;
      changed = true;
    } else if (c == '0') {
      paused = true;
      element = track->begin();
      changed = true;
    } else if (c == 'j') {
      // If we weren't paused, we are now.
      paused = true;

      // Move to next frame.
      ++element;
      if (element == track->end()) {
        element = track->begin();
      }
      changed = true;
    } else if (c == 'k') {
      // If we weren't paused, we are now.
      paused = true;

      // Move to previous frame.
      if (element == track->begin()) {
        element = track->end();
      }
      --element;
      changed = true;
    } else {
      if (paused) {
        if (c == 'D') {
          // Delete entire track.
          tracks.erase(track++);
          CHECK(!tracks.empty()) << "You deleted every point of every track";

          if (track == tracks.end()) {
            --track;
          }
          element = track->begin();

          marker_set = false;
          changed = true;
        } else if (c == 'x') {
          // Delete current frame.
          track->erase(element++);

          if (track->empty()) {
            // Erase track.
            tracks.erase(track++);
            CHECK(!tracks.empty()) << "You deleted every point of every track";

            if (track == tracks.end()) {
              --track;
            }

            element = track->begin();
          } else {
            if (element == track->end()) {
              --element;
            }
          }

          marker_set = false;
          changed = true;
        } else if (c == 'v') {
          if (marker_set) {
            marker_set = false;
          } else {
            marker_set = true;
            marker_frame = t;
            marker_element = element;
          }
        } else if (c == 'd') {
          if (marker_set) {
            if (marker_frame > t) {
              std::swap(marker_element, element);
            }

            // Delete from marker frame to this frame.
            track->erase(marker_element, ++element);

            if (track->empty()) {
              // Erase track.
              LOG(INFO) << "Deleting track";
              tracks.erase(track++);

              CHECK(!tracks.empty()) << "You deleted every point of every track";

              if (track == tracks.end()) {
                --track;
                element = track->begin();
              }
            } else {
              if (element == track->end()) {
                --element;
              }
            }

            marker_set = false;
            changed = true;
          } else {
            LOG(INFO) << "No marker set";
          }
        }
      }
    }

    if (!paused) {
      // Move to next frame.
      ++element;
      if (element == track->end()) {
        element = track->begin();
      }
      changed = true;
    }
  }

  track_list.clear();
  std::copy(tracks.begin(), tracks.end(), std::back_inserter(track_list));

  ScaleSpacePositionWriter feature_writer;
  ok = saveTrackList(output_file, track_list, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
