#include "draw_matches.hpp"
#include "random_color.hpp"
#include "rigid_feature.hpp"
#include "rigid_warp.hpp"

const int PATCH_SIZE = 9;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

typedef std::vector<cv::KeyPoint> KeypointList;

// Converts a cv::KeyPoint to a RigidFeature.
RigidFeature keypointToRigidFeature(const cv::KeyPoint& keypoint) {
  double theta = keypoint.angle / 180. * CV_PI;
  return RigidFeature(keypoint.pt.x, keypoint.pt.y, keypoint.size, theta);
}

// Renders a keypoint on top of an image with a random color.
void drawKeypoint(cv::Mat& image,
                  const cv::KeyPoint& keypoint,
                  const cv::Scalar& color) {
  // Convert to our format.
  RigidFeature feature = keypointToRigidFeature(keypoint);

  // Warp is just for drawing. This feels weird.
  RigidWarp warp(PATCH_SIZE);
  warp.draw(image, feature.data(), PATCH_SIZE, color);
}

void drawMatches(const KeypointList& keypoints1,
                 const KeypointList& keypoints2,
                 const MatchList& matches,
                 cv::Mat& image1,
                 cv::Mat& image2,
                 cv::Mat& render) {
  MatchList::const_iterator match;
  for (match = matches.begin(); match != matches.end(); ++match) {
    // Generate a random color.
    cv::Scalar color = randomColor(SATURATION, BRIGHTNESS);

    // Draw the keypoint in each image.
    drawKeypoint(image1, keypoints1[match->first], color);
    drawKeypoint(image2, keypoints2[match->second], color);
  }

  // Initialize large side-by-side image with black background.
  int rows = std::max(image1.rows, image2.rows);
  int cols = image1.cols + image2.cols;
  render.create(rows, cols, cv::DataType<cv::Vec3b>::type);
  render = cv::Scalar::all(0);

  // Copy each image into output.
  cv::Mat dst;
  dst = render(cv::Range(0, rows), cv::Range(0, image1.cols));
  image1.copyTo(dst);
  dst = render(cv::Range(0, rows), cv::Range(image1.cols, cols));
  image2.copyTo(dst);
}

