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

#include "read_image.hpp"
#include "track.hpp"
#include "tracker.hpp"
#include "klt_tracker.hpp"


typedef std::map<int, cv::Point2d> FeatureList;
typedef FeatureList::value_type Feature;

Feature readFeatureFromFileNode(const cv::FileNode& node) {
  int id = (int)node["id"];
  double x = (double)node["x"];
  double y = (double)node["y"];

  return Feature(id, cv::Point2d(x, y));
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: " << argv[0]
              << " image-format tracks-file descriptors-file" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Example" << std::endl;
    std::cerr << argv[0]
              << " input/my-video/%03d.png output/my-video/tracks.yaml"
              << " output/my-video/descriptors.yaml" << std::endl;
    return 1;
  }

  std::string image_format = argv[1];
  std::string tracks_filename = argv[2];
  std::string desc_filename = argv[3];

  // Frame index.
  int n = 0;

  // Open tracks file for reading.
  cv::FileStorage tracks_file(tracks_filename, cv::FileStorage::READ);
  if (!tracks_file.isOpened()) {
    std::cerr << "unable to open tracks file" << std::endl;
    return 1;
  }

  // Open descriptors file for writing.
  cv::FileStorage desc_file(desc_filename, cv::FileStorage::WRITE);
  if (!desc_file.isOpened()) {
    std::cerr << "unable to open descriptors file" << std::endl;
    return 1;
  }

  // Iterate through tracks file.
  cv::FileNode frames = tracks_file["tracks"];
  cv::FileNodeIterator frame;
  for (frame = frames.begin(); frame != frames.end(); ++frame) {
    // Build image filename.
    std::string input_filename;
    input_filename = boost::str(boost::format(image_format) % (n + 1));
    // Load image.
    cv::Mat color_image;
    cv::Mat image;
    bool ok = readImage(input_filename, color_image, image);
    if (!ok) {
      throw std::runtime_error("ran out of images");
    }

    // Load feature positions from tracks file.
    FeatureList features;
    std::transform((*frame).begin(), (*frame).end(),
        std::inserter(features, features.begin()), readFeatureFromFileNode);

    /*
    // Extract SIFT descriptor at those points.
    cv::DescriptorExtractor extractor;
    extractor.create("SIFT");
    cv::Mat descriptors;
    extractor.compute(image, points, descriptors);
    */

    n += 1;
  }

  return 0;
}
