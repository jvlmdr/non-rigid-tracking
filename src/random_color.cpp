#include "random_color.hpp"
#include <cstdlib>

// Returns a random color with given saturation and brightness.
// http://en.wikipedia.org/wiki/HSL_and_HSV
cv::Scalar randomColor(double saturation, double brightness) {
  double h = double(std::rand()) / RAND_MAX;
  double s = saturation;
  double v = brightness;

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
