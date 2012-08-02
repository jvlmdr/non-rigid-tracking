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
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "descriptor.hpp"
#include "random_color.hpp"

const int NUM_DIMENSIONS = 2;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " tracks-file" << std::endl;
    std::cerr << std::endl;
    std::cerr << "features must have a \"descriptor\" attribute" << std::endl;
    return 1;
  }

  std::string tracks_file = argv[1];

  typedef TrackList_<Descriptor> DescriptorTrackList;
  typedef Track_<Descriptor> DescriptorTrack;

  // Load tracks.
  DescriptorTrackList descriptor_tracks;
  ReadDescriptor read_descriptor;
  bool ok = descriptor_tracks.load(tracks_file, read_descriptor);

  if (!ok) {
    std::cerr << "could not load tracks" << std::endl;
    return 1;
  }

  // Number of tracks.
  int num_tracks = descriptor_tracks.size();
  // Number of observations.
  int num_descriptors = descriptor_tracks.countPoints();

  // Put all descriptors into one big matrix.
  cv::Mat descriptors = cv::Mat_<double>(num_descriptors, 128);
  {
    // Maintain overall descriptor index.
    int i = 0;
    DescriptorTrackList::const_iterator track;

    for (track = descriptor_tracks.begin();
         track != descriptor_tracks.end();
         ++track) {
      // Iterate through frames in each track.
      DescriptorTrack::const_iterator pair;
      for (pair = track->begin();
           pair != track->end();
           ++pair) {
        // Copy each descriptor into the matrix.
        const Descriptor& descriptor = pair->second;
        std::copy(descriptor.data.begin(), descriptor.data.end(),
            descriptors.row(i).begin<double>());

        i += 1;
      }
    }
  }

  // Perform PCA.
  cv::PCA pca(descriptors, cv::Mat(), CV_PCA_DATA_AS_ROW, NUM_DIMENSIONS);

  // Project each descriptor down on to the basis.
  cv::Mat alpha;
  pca.project(descriptors, alpha);

  std::cout << descriptors << std::endl;
  std::cout << alpha << std::endl;

  // Write out points in gnuplot format.
  {
    // Maintain overall descriptor index.
    int i = 0;
    DescriptorTrackList::const_iterator track;

    for (track = descriptor_tracks.begin();
         track != descriptor_tracks.end();
         ++track) {
      // Generate a random color.
      cv::Scalar rgb = randomColor(SATURATION, BRIGHTNESS);
      int b = rgb[0];
      int g = rgb[1];
      int r = rgb[2];
      int color = (((r << 8) | g) << 8) | b;

      // Iterate through frames in each track.
      DescriptorTrack::const_iterator descriptor;
      for (descriptor = track->begin();
           descriptor != track->end();
           ++descriptor) {
        for (int j = 0; j < NUM_DIMENSIONS; j += 1) {
          std::cout << alpha.at<double>(i, j);
          std::cout << "\t";
        }
        std::cout << color;
        std::cout << std::endl;

        i += 1;
      }
      std::cout << std::endl;
    }
  }

  return 0;
}
