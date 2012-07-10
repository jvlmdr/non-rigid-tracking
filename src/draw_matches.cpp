#include "draw_matches.hpp"

typedef std::vector<cv::KeyPoint> KeyPointList;

cv::Scalar randomColor() {
  // http://en.wikipedia.org/wiki/HSL_and_HSV
  double h = double(std::rand()) / RAND_MAX;
  double s = 0.99;
  double v = 0.99;

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

void drawMatches(const KeyPointList& keypoints1,
                 const KeyPointList& keypoints2,
                 const MatchList& matches,
                 cv::Mat& image1,
                 cv::Mat& image2,
                 cv::Mat& render) {
  MatchList::const_iterator match;
  for (match = matches.begin(); match != matches.end(); ++match) {
    // Generate a random color.
    cv::Scalar color = randomColor();

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

