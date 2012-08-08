#include <iostream>
#include <opencv2/core/core.hpp>
#include <boost/bind.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "image_point_reader.hpp"
#include "image_point_writer.hpp"
#include "vector_reader.hpp"
#include "vector_writer.hpp"
#include "camera_properties_reader.hpp"

cv::Point2d subtract(const cv::Point2d& x, const cv::Point2d& y) {
  return x - y;
}

cv::Point2d undistort(const cv::Point2d& x, double w) {
  double rd = cv::norm(x);
  double ru = std::tan(rd * w) / (2. * std::tan(w / 2.));

  return ru / rd * x;
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Removes lens distortion from a set of points." << std::endl;
  usage << std::endl;
  usage << argv[0] << " distorted-points undistorted-points camera-properties" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
}

int main(int argc, char** argv) {
  init(argc, argv);

  // Read required parameters.
  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }
  std::string distorted_points_file = argv[1];
  std::string undistorted_points_file = argv[2];
  std::string camera_file = argv[3];

  bool ok;

  // Load points.
  std::vector<cv::Point2d> points;
  ImagePointReader point_reader;
  ok = loadList(distorted_points_file, points, point_reader);

  // Load camera properties.
  CameraProperties camera;
  CameraPropertiesReader camera_reader;
  ok = load(camera_file, camera, camera_reader);

  // Intrinsics are applied effectively in this order:
  //   diag(fx, fy) (x, y) + (x0, y0)
  // Therefore undo principal point, then undistort.

  // Subtract principal point.
  std::transform(points.begin(), points.end(), points.begin(),
      boost::bind(subtract, _1, camera.principal_point));

  // TODO: Flip Y axis? Depends how camera matrices are defined.

  // Undistort.
  std::transform(points.begin(), points.end(), points.begin(),
      boost::bind(undistort, _1, camera.distort_w));

  // Write out undistorted points.
  ImagePointWriter point_writer;
  saveList(undistorted_points_file, points, point_writer);

  return 0;
}
