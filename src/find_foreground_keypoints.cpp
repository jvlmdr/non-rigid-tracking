#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/video/video.hpp>

#include "read_image.hpp"

// Maximum number of features. 0 for no limit.
const int MAX_NUM_FEATURES = 0;
// Background subtraction parameters.
const int HISTORY_LENGTH = 120;
// Variance threshold in sigmas.
const double OUTLIER_VARIANCE = 3.0;
// Number of components in Gaussian mixture model.
const int MAX_MIXTURE_COMPONENTS = 1;

std::string imageFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

std::string keypointsFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

bool writeKeyPointToFile(cv::FileStorage& out, const cv::KeyPoint& k) {
  // SIFT does not set 'response' or 'class_id' attributes.
  out << "{:";
  out << "x" << k.pt.x << "y" << k.pt.y;
  out << "size" << k.size;
  out << "angle" << k.angle;
  out << "octave" << k.octave;
  out << "}";

  return true;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " image-format keypoints-format" <<
      std::endl;
    std::cerr << std::endl;
    std::cerr << "Example" << std::endl;
    std::cerr << argv[0] <<
      " input/my-video/%07d.png output/my-video/keypoints/%07d.yaml" <<
      std::endl;
    return 1;
  }

  std::string image_format = argv[1];
  std::string keypoints_format = argv[2];

  // Loop variables.
  int n = 0;
  bool have_frame = true;
  cv::Mat image;
  cv::Mat color_image;
  cv::Mat image_double;
  cv::Mat total;

  /*
  // First pass: Compute mean.
  while (have_frame) {
    // Attempt to read next frame.
    have_frame = readImage(imageFilename(image_format, n), color_image, image);
    if (!have_frame) {
      continue;
    }

    // Convert to floating point.
    image.convertTo(image_double, cv::DataType<double>::type, 1. / 255.);

    // Initialize total if required.
    if (total.empty()) {
      total = cv::Mat_<double>(image.size(), 0.);
    }
    total += image_double;

    // Frame counter.
    n += 1;

    cv::imshow("mean", total / n);
    cv::waitKey(10);
  }

  int num_frames = n;
  cv::Mat mean = total / num_frames;

  // Second pass: Compute variance.
  total = cv::Mat();
  cv::Mat difference;

  for (n = 0; n < num_frames; n += 1) {
    // Attempt to read next frame.
    have_frame = readImage(imageFilename(image_format, n), color_image, image);
    if (!have_frame) {
      throw std::runtime_error("ran out of frames");
    }

    // Convert to floating point.
    image.convertTo(difference, cv::DataType<double>::type, 1. / 255.);
    // Subtract mean.
    difference -= mean;
    // Square.
    cv::multiply(difference, difference, difference);

    // Initialize total if required.
    if (total.empty()) {
      total = cv::Mat_<double>(image.size(), 0.);
    }
    total += difference;

    cv::imshow("variance", total / n);
    cv::waitKey(10);
  }

  cv::Mat variance = total / num_frames;

  {
    cv::FileStorage fs("buffer.yaml", cv::FileStorage::WRITE);
    fs << "num_frames" << num_frames;
    fs << "mean" << mean;
    fs << "variance" << variance;
  }
  */

  int num_frames;
  cv::Mat mean;
  cv::Mat variance;

  {
    cv::FileStorage fs("buffer.yaml", cv::FileStorage::READ);
    num_frames = (int)fs["num_frames"];
    fs["mean"] >> mean;
    fs["variance"] >> variance;
  }

  // Third pass: Threshold foreground.
  cv::Mat foreground;

  for (n = 0; n < num_frames; n += 1) {
    // Attempt to read next frame.
    have_frame = readImage(imageFilename(image_format, n), color_image, image);
    if (!have_frame) {
      throw std::runtime_error("ran out of frames");
    }

    // Convert to floating point.
    image.convertTo(foreground, cv::DataType<double>::type, 1. / 255.);

    // Subtract mean.
    foreground -= mean;
    // Square.
    cv::multiply(foreground, foreground, foreground);
    // Divide by sigma^2.
    cv::divide(foreground, variance, foreground);
    // Square root.
    cv::sqrt(foreground, foreground);

    const int erode_radius = 6;
    const int dilate_radius = 32;
    cv::Mat kernel;

    kernel = cv::Mat_<uint8_t>(2 * erode_radius + 1, 2 * erode_radius + 1,
        uint8_t(0));
    cv::circle(kernel, cv::Point(erode_radius, erode_radius), erode_radius,
        cv::Scalar(1), -1);
    cv::erode(foreground, foreground, kernel);

    kernel = cv::Mat_<uint8_t>(2 * dilate_radius + 1, 2 * dilate_radius + 1,
        uint8_t(0));
    cv::circle(kernel, cv::Point(dilate_radius, dilate_radius), dilate_radius,
        cv::Scalar(1), -1);
    cv::dilate(foreground, foreground, kernel);

    cv::imshow("foreground", foreground >= 2);
    cv::waitKey(10);
  }

  return 0;
}
