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
#include "track_list.hpp"
#include "rigid_feature.hpp"
#include "rigid_warp.hpp"
#include "random_color.hpp"

const int PATCH_SIZE = 9;
const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

DEFINE_string(output_format, "%d.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

void drawFeatures(cv::Mat& image,
                  const std::map<int, RigidFeature>& features,
                  const std::vector<cv::Scalar>& colors) {
  typedef std::map<int, RigidFeature> FeatureSet;
  typedef std::vector<cv::Scalar> ColorList;

  RigidWarp warp(PATCH_SIZE);

  FeatureSet::const_iterator mapping;
  for (mapping = features.begin(); mapping != features.end(); ++mapping) {
    int index = mapping->first;
    const RigidFeature& feature = mapping->second;

    warp.draw(image, feature.data(), PATCH_SIZE, colors[index]);
  }
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Visualizes rigid-warp tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks image-format";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];

  bool ok;

  // Load tracks.
  TrackList_<RigidFeature> tracks;
  ReadRigidFeature read;
  ok = tracks.load(tracks_file, read);
  if (!ok) {
    std::cerr << "could not load tracks" << std::endl;
    return 1;
  }

  // Make a list of random colors.
  typedef std::vector<cv::Scalar> ColorList;
  ColorList colors;
  for (int i = 0; i < int(tracks.size()); i += 1) {
    colors.push_back(randomColor(BRIGHTNESS, SATURATION));
  }

  // Iterate through frames in which track was observed.
  FrameIterator_<RigidFeature> frame(tracks);
  frame.seekToStart();

  while (!frame.end()) {
    // Get the current time.
    int t = frame.t();

    // Load the image.
    cv::Mat color_image;
    cv::Mat gray_image;
    ok = readImage(makeFilename(image_format, t), color_image, gray_image);
    if (!ok) {
      std::cerr << "could not read image" << std::endl;
      return 1;
    }

    // Get the features.
    typedef std::map<int, RigidFeature> FeatureSet;
    FeatureSet features;
    frame.getPoints(features);

    // Draw each one with its color.
    drawFeatures(color_image, features, colors);

    cv::imshow("frame", color_image);
    cv::waitKey(0);

    ++frame;
  }

  return 0;
}
