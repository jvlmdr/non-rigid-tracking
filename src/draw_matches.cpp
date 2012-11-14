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
                 const cv::Mat& image1,
                 const cv::Mat& image2,
                 cv::Mat& render,
                 int line_thickness) {
  // Initialize large side-by-side image with black background.
  int rows = std::max(image1.rows, image2.rows);
  int cols = image1.cols + image2.cols;
  render.create(rows, cols, cv::DataType<cv::Vec3b>::type);
  render = cv::Scalar::all(0);

  cv::Mat viewport1 = render(cv::Range(0, image1.rows),
                             cv::Range(0, image1.cols));
  cv::Mat viewport2 = render(cv::Range(0, image2.rows),
                             cv::Range(image1.cols, cols));

  // Copy images into viewports.
  image1.copyTo(viewport1);
  image2.copyTo(viewport2);

  MatchList::const_iterator match;
  for (match = matches.begin(); match != matches.end(); ++match) {
    // Generate a random color.
    cv::Scalar color = randomColor(SATURATION, BRIGHTNESS);

    SiftPosition keypoint1 = keypoints1[match->first];
    SiftPosition keypoint2 = keypoints2[match->second];

    // Draw the keypoint in each image.
    drawSiftPosition(viewport1, keypoint1, color, line_thickness);
    drawSiftPosition(viewport2, keypoint2, color, line_thickness);

    // Draw line between two keypoints.
    cv::Point pt1 = keypoint1.point();
    cv::Point pt2(keypoint2.x + image1.cols, keypoint2.y);
    cv::line(render, pt1, pt2, color, line_thickness);
  }
}
