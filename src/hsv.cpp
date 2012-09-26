#include "hsv.hpp"

cv::Scalar hsvToRgb(double h, double s, double v) {
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

void evenlySpacedColors(int n,
                        double s,
                        double v,
                        std::vector<cv::Scalar>& colors) {
  colors.clear();

  for (int i = 0; i < n; i += 1) {
    double h = double(i) / n;
    colors.push_back(hsvToRgb(h, s, v));
  }
}
