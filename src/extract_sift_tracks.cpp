#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "read_image.hpp"
#include "warp.hpp"
#include "rigid_warp.hpp"
#include "rigid_feature.hpp"
#include "descriptor.hpp"
#include "sift.hpp"

// Size of window to track.
const int PATCH_SIZE = 9;
const int NUM_PIXELS = PATCH_SIZE * PATCH_SIZE;

// Lucas-Kanade optimization settings.
const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-4;
const double GRADIENT_TOLERANCE = 0;
const double PARAMETER_TOLERANCE = 1e-4;
const bool ITERATION_LIMIT_IS_FATAL = true;
const double MAX_CONDITION = 100;

const int SIFT_FIXPT_SCALE = 48;
// assumed gaussian blur for input image
const float SIFT_INIT_SIGMA = 0.5f;

// Do not want to sample below one pixel.
const double MIN_SCALE = 0.5;

// Maximum average intensity difference as a fraction of the range.
// (A value of 1 means anything is permitted.)
const double MAX_RESIDUAL = 0.1;

const int MAX_NUM_FEATURES = 100;
const int NUM_OCTAVE_LAYERS = 3;
const double CONTRAST_THRESHOLD = 0.04;
const double EDGE_THRESHOLD = 10;
const double SIGMA = 1.6;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

class ReadRigidFeature : public Read<RigidFeature> {
  public:
    ~ReadRigidFeature() {}

    void operator()(const cv::FileNode& node, RigidFeature& feature) {
      feature.x = static_cast<double>(node["x"]);
      feature.y = static_cast<double>(node["y"]);
      feature.size = static_cast<double>(node["size"]);
      feature.theta = static_cast<double>(node["angle"]);
    }
};

struct Feature {
  // This is "position" in a general sense. More like 2D pose.
  RigidFeature position;
  // Fixed-size representation of appearance.
  Descriptor descriptor;
};

class WriteFeature : public Write<Feature> {
  public:
    ~WriteFeature() {}

    void operator()(cv::FileStorage& file, const Feature& feature) {
      file << "{";
      file << "x" << feature.position.x;
      file << "y" << feature.position.y;
      file << "size" << feature.position.size;
      file << "angle" << feature.position.theta;
      file << "descriptor";
      feature.descriptor.write(file);
      file << "}";
    }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 4) {
    std::cerr << "usage: " << argv[0] << " tracks-file image-format"
      " descriptors-file" << std::endl;
    return 1;
  }

  std::string positions_file = argv[1];
  std::string image_format = argv[2];
  std::string descriptors_file = argv[3];

  // Load tracks.
  TrackList_<RigidFeature> position_tracks;
  ReadRigidFeature read_position;
  position_tracks.load(positions_file, read_position);

  int num_features = position_tracks.size();

  // Where to put the result.
  TrackList_<Feature> feature_tracks(num_features);

  // Iterate over each frame in the track.
  FrameIterator_<RigidFeature> frame(position_tracks);
  frame.seekToStart();

  while (!frame.end()) {
    // Get features in this frame.
    typedef std::map<int, RigidFeature> FeatureSet;
    FeatureSet positions;
    frame.getPoints(positions);

    int t = frame.t();
    std::cout << "frame " << t << ": " << positions.size() << " features" <<
      std::endl;

    // Load image.
    std::string image_file = makeFilename(image_format, t);
    cv::Mat integer_image;
    cv::Mat color_image;
    bool ok = readImage(image_file, color_image, integer_image);
    if (!ok) {
      std::cerr << "could not read image" << std::endl;
      return 1;
    }

    SiftExtractor sift(integer_image, NUM_OCTAVE_LAYERS, SIGMA);

    // Extract descriptor for each and store in track.
    for (FeatureSet::const_iterator it = positions.begin();
         it != positions.end();
         ++it) {
      int i = it->first;
      const RigidFeature& position = it->second;

      Feature& feature = (feature_tracks[i][t] = Feature());
      feature.position = position;
      sift.extractDescriptor(position, feature.descriptor);
    }

    ++frame;
  }

  WriteFeature write;
  feature_tracks.save(descriptors_file, write);

  return 0;
}
