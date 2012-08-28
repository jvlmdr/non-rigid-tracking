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
#include "track_list.hpp"
#include "warp.hpp"
#include "similarity_feature.hpp"
#include "descriptor.hpp"
#include "sift.hpp"
#include "similarity_feature_reader.hpp"
#include "track_list_reader.hpp"
#include "writer.hpp"
#include "similarity_feature_writer.hpp"
#include "descriptor_writer.hpp"
#include "track_list_writer.hpp"

const int NUM_OCTAVE_LAYERS = 3;
const double SIGMA = 1.6;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

struct Feature {
  // This is "position" in a general sense. More like 2D pose.
  SimilarityFeature position;
  // Fixed-size representation of appearance.
  Descriptor descriptor;
};

class FeatureWriter : public Writer<Feature> {
  public:
    ~FeatureWriter() {}

    void write(cv::FileStorage& file, const Feature& feature) {
      SimilarityFeatureWriter position_writer;
      position_writer.write(file, feature.position);

      DescriptorWriter descriptor_writer;
      descriptor_writer.write(file, feature.descriptor);
    }
};

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Extracts SIFT descriptors at every position in a track." <<
      std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " tracks-file image-format descriptors-file" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string positions_file = argv[1];
  std::string image_format = argv[2];
  std::string descriptors_file = argv[3];

  // Load tracks.
  TrackList<SimilarityFeature> position_tracks;
  SimilarityFeatureReader position_reader;
  bool ok = loadTrackList(positions_file, position_tracks, position_reader);
  CHECK(ok) << "Could not load tracks";

  int num_features = position_tracks.size();

  // Where to put the result.
  TrackList<Feature> feature_tracks(num_features);

  // Iterate over each frame in the track.
  TrackListTimeIterator<SimilarityFeature> frame(position_tracks);
  frame.seekToStart();

  while (!frame.end()) {
    // Get features in this frame.
    typedef std::map<int, SimilarityFeature> FeatureSet;
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
      const SimilarityFeature& position = it->second;

      Feature& feature = (feature_tracks[i][t] = Feature());
      feature.position = position;
      sift.extractDescriptor(position, feature.descriptor);
    }

    ++frame;
  }

  FeatureWriter feature_writer;
  ok = saveTrackList(descriptors_file, feature_tracks, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
