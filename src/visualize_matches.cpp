#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cstdlib>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "read_image.hpp"
#include "descriptor.hpp"
#include "keypoint.hpp"
#include "match.hpp"

typedef std::vector<cv::KeyPoint> KeyPointList;

cv::Scalar randomColor() {
  double h = double(std::rand()) / RAND_MAX;
  double s = 0.8;
  double v = 0.9;

  double c = v * s;
  double h_dash = 6 * h;
  double x = c * (1 - std::abs(1 - std::fmod(h_dash, 2)));

  cv::Scalar rgb;
  if (h_dash < 1) {
    rgb = cv::Scalar(c, x, 0);
  } else if (h_dash < 2) {
    rgb = cv::Scalar(x, c, 0);
  } else if (h_dash < 3) {
    rgb = cv::Scalar(0, c, x);
  } else if (h_dash < 4) {
    rgb = cv::Scalar(0, x, c);
  } else if (h_dash < 5) {
    rgb = cv::Scalar(x, 0, c);
  } else if (h_dash < 6) {
    rgb = cv::Scalar(c, 0, x);
  } else {
    rgb = cv::Scalar(0, 0, 0);
  }

  double m = v - c;
  rgb += cv::Scalar(m, m, m);
  rgb *= 255;

  return rgb;
}

void printUsage(std::ostream& out, const std::string& name) {
  out << "Usage: " << name << " matches image1 image2 keypoints1 keypoints2 output" <<
    std::endl;
  out << std::endl;
}

int main(int argc, char** argv) {
  if (argc < 7) {
    printUsage(std::cerr, argv[0]);
    return 1;
  }

  std::string matches_file = argv[1];
  std::string image_file1 = argv[2];
  std::string image_file2 = argv[3];
  std::string keypoints_file1 = argv[4];
  std::string keypoints_file2 = argv[5];
  std::string output_file = argv[6];

  bool ok;

  // Load matches.
  MatchList matches;
  ok = loadMatches(matches_file, matches);
  if (!ok) {
    std::cerr << "could not load matches" << std::endl;
    return 1;
  }

  // Load images.
  cv::Mat image1;
  cv::Mat image2;
  cv::Mat gray;
  ok = readImage(image_file1, image1, gray);
  if (!ok) {
    std::cerr << "could not load image" << std::endl;
    return 1;
  }
  ok = readImage(image_file2, image2, gray);
  if (!ok) {
    std::cerr << "could not load image" << std::endl;
    return 1;
  }

  // Load keypoints.
  KeyPointList keypoints1;
  KeyPointList keypoints2;
  ok = loadKeyPoints(keypoints_file1, keypoints1);
  if (!ok) {
    std::cerr << "could not load keypoints" << std::endl;
    return 1;
  }
  ok = loadKeyPoints(keypoints_file2, keypoints2);
  if (!ok) {
    std::cerr << "could not load keypoints" << std::endl;
    return 1;
  }

  // Generate some random colors to show the matches.
  int num_matches = matches.size();
  std::vector<cv::Scalar> colors;

  MatchList::const_iterator match;
  for (match = matches.begin(); match != matches.end(); ++match) {
    // Generate a random color.
    cv::Scalar color = randomColor();

    // Draw the match position.
    const cv::KeyPoint* keypoint;

    keypoint = &keypoints1[match->first];
    cv::circle(image1, keypoint->pt, 8, color, 2);

    keypoint = &keypoints2[match->second];
    cv::circle(image2, keypoint->pt, 8, color, 2);
  }

  int rows = image1.rows;
  int cols = image1.cols + image2.cols;
  cv::Mat render = cv::Mat_<cv::Vec3b>(rows, cols, cv::Vec3b(0, 0, 0));

  cv::Mat dst;
  dst = render(cv::Range(0, rows), cv::Range(0, image1.cols));
  image1.copyTo(dst);
  dst = render(cv::Range(0, rows), cv::Range(image1.cols, cols));
  image2.copyTo(dst);

  cv::imwrite(output_file, render);

  return 0;
}
