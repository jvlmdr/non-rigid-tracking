#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "track_list.hpp"
#include "read_image.hpp"
#include "descriptor.hpp"

// Default SIFT feature size.
// 1.6 from cv::SIFT constructor.
// Multiply by two from SIFT::adjustLocalExtrema.
const double FEATURE_SIZE = 2 * 1.6;
const double FEATURE_ANGLE = 0;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

cv::KeyPoint makeKeypoint(const FrameIterator::Points::value_type& indexed) {
  const cv::Point2d& point = indexed.second;
  return cv::KeyPoint(point, FEATURE_SIZE, FEATURE_ANGLE);
}

void printUsage(std::ostream& out, const std::string& name) {
  out << "usage: " << name << " tracks image-format descriptors" <<
    std::endl;
  out << std::endl;

  out << "Parameters:" << std::endl;
  out << "tracks -- Input. File containing 2D tracks." << std::endl;
  out << "image-format -- Input. Format for frame images." << std::endl;
  out << "descriptors -- Output. Will contain a descriptor for each"
    " point." << std::endl;
  out << std::endl;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    printUsage(std::cerr, argv[0]);
    return 1;
  }

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];
  std::string descriptors_file = argv[3];

  // Attempt to load tracks.
  TrackList tracks;
  cv::Size size;
  int num_frames;
  bool ok = loadTracks(tracks_file, size, tracks, &num_frames);
  if (!ok) {
    std::cerr << "could not load tracks file" << std::endl;
    return 1;
  }

  // Iterate through frames.
  FrameIterator frame(tracks);
  int t = 0;

  // Construct a list of empty track descriptors.
  int num_tracks = tracks.size();
  TrackList_<Descriptor> descriptor_lists(num_tracks);

  while (!frame.end() && ok) {
    // Extract points.
    FrameIterator::Points points;
    frame.getPoints(points);

    if (!points.empty()) {
      // Load image.
      cv::Mat image;
      cv::Mat color_image;
      std::string image_file = makeFilename(image_format, t);
      ok = readImage(image_file, color_image, image);
      if (!ok) {
        std::cerr << "unable to load image for frame " << t << std::endl;
        return 1;
      }

      // Roll your own keypoints without a feature detector.
      std::vector<cv::KeyPoint> keypoints;
      std::transform(points.begin(), points.end(),
          std::back_inserter(keypoints), makeKeypoint);

      // Extract descriptors at these points.
      cv::Mat table;
      cv::SIFT sift;
      sift(image, cv::noArray(), keypoints, table, true);

      // Make sure that it return 128-D vectors of 32-bit floats as we expect.
      if (table.type() != cv::DataType<float>::type) {
        std::cerr << "expect SIFT to return 32-bit floats" << std::endl;
        return 1;
      }
      if (table.cols != 128) {
        std::cerr << "expect SIFT to return 128-D vectors" << std::endl;
        return 1;
      }

      // Copy into nicer data structure.
      typedef std::vector<Descriptor> DescriptorList;
      DescriptorList descriptors;
      for (int i = 0; i < table.rows; i += 1) {
        // Add a descriptor to the vector.
        descriptors.push_back(Descriptor());

        // Copy row into descriptor.
        cv::Mat row = table.row(i);
        std::copy(row.begin<float>(), row.end<float>(),
            std::back_inserter(descriptors.back().data));
      }

      // Copy into list of descriptors for whole tracks.
      FrameIterator::Points::const_iterator point = points.begin();
      DescriptorList::const_iterator descriptor = descriptors.begin();

      while (point != points.end()) {
        int index = point->first;
        descriptor_lists[index][t] = *descriptor;
        ++point;
        ++descriptor;
      }
    }

    ++frame;
    t += 1;
  }

  WriteDescriptor write;
  descriptor_lists.save(descriptors_file, write);

  return 0;
}
