#include "draw_matches.hpp"
#include "random_color.hpp"
#include "sift_position.hpp"
#include "draw_sift_position.hpp"

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

typedef std::vector<SiftPosition> SiftPositionList;
typedef std::vector<Match> MatchList;

void drawMatches(const std::vector<SiftPosition>& keypoints1,
                 const std::vector<SiftPosition>& keypoints2,
                 const std::vector<Match>& matches,
                 cv::Mat& image1,
                 cv::Mat& image2,
                 cv::Mat& render,
                 int line_thickness) {
  MatchList::const_iterator match;
  for (match = matches.begin(); match != matches.end(); ++match) {
    // Generate a random color.
    cv::Scalar color = randomColor(SATURATION, BRIGHTNESS);

    // Draw the keypoint in each image.
    drawSiftPosition(image1, keypoints1[match->first], color, line_thickness);
    drawSiftPosition(image2, keypoints2[match->second], color, line_thickness);
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

