#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/array.hpp>
#include <boost/scoped_array.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "read_image.hpp"
#include "track.hpp"
#include "tracker.hpp"
#include "klt_tracker.hpp"

void printUsage(std::ostream& out, const std::string& name) {
  out << "usage: " << name << " image keypoints descriptors" << std::endl;
  out << std::endl;

  out << "Parameters:" << std::endl;
  out << "image -- Image file." << std::endl;
  out << "keypoints -- Input." <<
    " File containing keypoints at scale-space maxima." << std::endl;
  out << "  Includes scale, orientation and octave data." << std::endl;
  out << "descriptors -- Output.";
  out << " Will contain a descriptor for each keypoint." << std::endl;
  out << std::endl;

  out << "Example:" << std::endl;
  out << name << " input/005.png output/keypoints/005.yaml"
    " output/descriptors/005.yaml" << std::endl;
  out << std::endl;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    printUsage(std::cerr, argv[0]);
    return 1;
  }

  std::string image_filename = argv[1];
  std::string keypoints_filename = argv[2];
  std::string descriptors_filename = argv[3];

  // Read in image.
  cv::Mat color_image;
  cv::Mat image;
  bool ok = readImage(image_filename, color_image, image);
  if (!ok) {
    std::cerr << "unable to load image" << std::endl;
    return 1;
  }

  // Load keypoints.
  std::vector<cv::KeyPoint> keypoints;
  ok = loadKeyPoints(keypoints_filename, keypoints);
  if (!ok) {
    std::cerr << "unable to read keypoints" << std::endl;
    return 1;
  }
  int num_features = keypoints.size();

  // Extract SIFT descriptors at SIFT keypoints.
  // It is critical that the SIFT settings here match the detector.
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
  std::vector<Descriptor> descriptors(num_features);
  for (int i = 0; i < table.rows; i += 1) {
    // Add a descriptor to the vector.
    descriptors.push_back(Descriptor());
    Descriptor& descriptor = descriptors.back();

    // Copy row into descriptor.
    cv::Mat row = table.row(i);
    std::copy(row.begin<float>(), row.end<float>(), descriptor.begin());
  }

  // Write out descriptors.
  ok = saveDescriptors(descriptors_filename, descriptors);
  if (!ok) {
    std::cerr << "unable to write descriptors to file" << std::endl;
    return 1;
  }

  return 0;
}
