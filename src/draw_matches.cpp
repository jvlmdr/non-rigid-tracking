#include "draw_matches.hpp"
#include "random_color.hpp"

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

typedef std::vector<cv::KeyPoint> KeyPointList;

void drawMatches(const KeyPointList& keypoints1,
                 const KeyPointList& keypoints2,
                 const MatchList& matches,
                 cv::Mat& image1,
                 cv::Mat& image2,
                 cv::Mat& render) {
  MatchList::const_iterator match;
  for (match = matches.begin(); match != matches.end(); ++match) {
    // Generate a random color.
    cv::Scalar color = randomColor(SATURATION, BRIGHTNESS);

    // Draw the keypoint in each image.
    const cv::KeyPoint* keypoint;
    std::vector<cv::KeyPoint> single;

    keypoint = &keypoints1[match->first];
    single.assign(1, *keypoint);
    cv::drawKeypoints(image1, single, image1, color,
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    keypoint = &keypoints2[match->second];
    single.assign(1, *keypoint);
    cv::drawKeypoints(image2, single, image2, color,
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
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

