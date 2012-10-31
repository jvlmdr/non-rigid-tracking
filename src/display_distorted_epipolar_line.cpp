#include <string>
#include <vector>
#include <set>
#include <stack>
#include <numeric>
#include <sstream>
#include <cstdlib>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camera_properties.hpp"
#include "sift_position.hpp"
#include "distortion.hpp"
#include "util.hpp"

#include "iterator_reader.hpp"
#include "sift_position_reader.hpp"
#include "camera_properties_reader.hpp"
#include "matrix_reader.hpp"

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Visualizes distorted epipolar lines" << std::endl;
  usage << std::endl;
  usage << argv[0] << " keypoints intrinsics1 intrinsics2 fund-mat image1 "
      "image2" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 7) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

// Solves:
// x1^2 + x2^2 = r^2          (1)
// e1 x1 + e2 x2 + e3 = 0     (2)
std::vector<cv::Point2d> lineCircleIntersection(double r,
                                                double e1,
                                                double e2,
                                                double e3) {
  std::vector<cv::Point2d> solutions;

  // x2 = -(e1 x1 + e3) / e2
  // x1^2 + x2^2 = r^2
  // x1^2 + ((e1 x1 + e3) / e2)^2 = r^2
  // e2^2 x1^2 + (e1 x1 + e3)^2 = e2^2 r^2
  // (e2^2 + e1^2) x1^2 + 2 e1 e3 x1 + e3^2 - e2^2 r^2 = 0
  double a = e1 * e1 + e2 * e2;
  double b = 2 * e1 * e3;
  double c = e3 * e3 - e2 * e2 * r * r;

  double delta = b * b - 4. * a * c;
  if (delta == 0) {
    double u = -b / (2. * a);
    double v = -(e1 * u + e3) / e2;
    solutions.push_back(cv::Point2d(u, v));
  } else if (delta >= 0) {
    double u;
    double v;

    u = (-b + std::sqrt(delta)) / (2. * a);
    // TODO: This may be unstable if e2 is close to zero?
    v = -(e1 * u + e3) / e2;
    solutions.push_back(cv::Point2d(u, v));

    u = (-b - std::sqrt(delta)) / (2. * a);
    v = -(e1 * u + e3) / e2;
    solutions.push_back(cv::Point2d(u, v));
  }

  return solutions;
}

bool fourConnected(const cv::Point2d& p, const cv::Point2d& q) {
  bool connected = false;

  if (p.x == q.x) {
    if (p.y - 1 <= q.y && q.y <= p.y + 1) {
      connected = true;
    }
  } else if (p.y == q.y) {
    if (p.x - 1 <= q.x && q.x <= p.x + 1) {
      connected = true;
    }
  }

  return connected;
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string keypoints_file = argv[1];
  std::string camera_file1 = argv[2];
  std::string camera_file2 = argv[3];
  std::string fund_mat_file = argv[4];

  bool ok;

  // Load camera properties.
  CameraProperties camera1;
  CameraProperties camera2;

  CameraPropertiesReader camera_reader;
  ok = load(camera_file1, camera1, camera_reader);
  CHECK(ok) << "Could not load intrinsics for first camera";
  ok = load(camera_file2, camera2, camera_reader);
  CHECK(ok) << "Could not load intrinsics for second camera";

  cv::Mat F;
  MatrixReader matrix_reader;
  ok = load(fund_mat_file, F, matrix_reader);
  CHECK(ok) << "Could not load fundamental matrix";

  std::vector<SiftPosition> keypoints;
  // Load keypoints.
  SiftPositionReader keypoint_reader;
  ok = loadList(keypoints_file, keypoints, keypoint_reader);
  CHECK(ok) << "Could not load keypoints";

  // Get intrinics matrix for first camera.
  cv::Mat K1(camera1.matrix());
  cv::Mat K2(camera2.matrix());

  // Get the four points at the bounds of the second image.
  std::vector<cv::Point2d> corners;
  int w = camera2.image_size.width;
  int h = camera2.image_size.width;
  corners.push_back(cv::Point2d(    0,     0));
  corners.push_back(cv::Point2d(w - 1,     0));
  corners.push_back(cv::Point2d(    0, h - 1));
  corners.push_back(cv::Point2d(w - 1, h - 1));

  // Convert to homogeneous co-ordinates.
  cv::Mat X;
  imagePointsToHomogeneous(corners, X);
  // Undo intrinsics.
  X = K2.inv() * X;
  // Convert back to inhomogeneous co-ordinates.
  imagePointsFromHomogeneous(X, corners);

  // Find radius of each point.
  std::vector<double> radii;
  double (*take_norm)(const cv::Point2d&) = cv::norm;
  std::transform(corners.begin(), corners.end(), std::back_inserter(radii),
      take_norm);

  // Find maximum radius.
  double distorted_radius = std::accumulate(radii.begin(), radii.end(), 0.,
      std::max<double>);

  double max_distorted_radius = 0.99 * maxDistortedRadius(camera2.distort_w);
  if (distorted_radius > max_distorted_radius) {
    LOG(INFO) << "Clipping radius from " << distorted_radius << " to " <<
        max_distorted_radius;
    distorted_radius = max_distorted_radius;
  }

  double undistorted_radius = undistortRadius(distorted_radius,
      camera2.distort_w);

  std::vector<SiftPosition>::const_iterator keypoint;
  for (keypoint = keypoints.begin(); keypoint != keypoints.end(); ++keypoint) {
    cv::Point2d x1(keypoint->x, keypoint->y);
    // Undo intrinsics, undistort, and re-apply intrinsics.
    x1 = imagePointFromHomogeneous(K1.inv() * imagePointToHomogeneous(x1));
    x1 = undistort(x1, camera1.distort_w);
    x1 = imagePointFromHomogeneous(K1 * imagePointToHomogeneous(x1));

    cv::Mat X1 = imagePointToHomogeneous(x1);

    // Find co-ordinates of epipolar line. Given an uncalibrated point in image
    // 1, we want an equation for the calibrated point in image 2.
    cv::Mat e = K2.t() * F * X1;
    // May as well normalize.
    e = e / cv::norm(e);

    // Find intersections.
    std::vector<cv::Point2d> solutions = lineCircleIntersection(
        undistorted_radius, e.at<double>(0, 0), e.at<double>(1, 0),
        e.at<double>(2, 0));

    if (solutions.size() == 2) {
      cv::Point2d a = solutions[0];
      cv::Point2d b = solutions[1];

      std::set<std::pair<int, int> > line;
      std::stack<std::pair<double, double> > intervals;
      intervals.push(std::pair<double, double>(0, 1));

      while (!intervals.empty()) {
        // Pull next element off the stack.
        double s = intervals.top().first;
        double t = intervals.top().second;
        intervals.pop();

        cv::Point2d u = (1 - s) * a + s * b;
        cv::Point2d v = (1 - t) * a + t * b;

        // Apply distortion to points.
        u = distort(u, camera2.distort_w);
        v = distort(v, camera2.distort_w);

        // Apply intrinsics to points.
        u = imagePointFromHomogeneous(K2 * imagePointToHomogeneous(u));
        v = imagePointFromHomogeneous(K2 * imagePointToHomogeneous(v));

        // Quantize to pixels.
        cv::Point p = u;
        cv::Point q = v;

        // Add to line.
        line.insert(std::pair<int, int>(p.x, p.y));
        line.insert(std::pair<int, int>(q.x, q.y));

        // Check if they are connected.
        if (!fourConnected(p, q)) {
          double mu = (s + t) / 2.;
          intervals.push(std::pair<double, double>(mu, t));
          intervals.push(std::pair<double, double>(s, mu));
        }
      }

      // Draw line.
      cv::Mat image = cv::Mat_<uint8_t>(camera2.image_size, 0);
      std::set<std::pair<int, int> >::const_iterator pixel;

      cv::Rect bounds(cv::Point(0, 0), camera2.image_size);

      for (pixel = line.begin(); pixel != line.end(); ++pixel) {
        cv::Point position(pixel->first, pixel->second);
        if (bounds.contains(position)) {
          image.at<uint8_t>(pixel->second, pixel->first) = 255;
        }
      }

      cv::imshow("line", image);
      cv::waitKey(10);
    }
  }

  return 0;
}
