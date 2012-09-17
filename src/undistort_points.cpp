#include <iostream>
#include <opencv2/core/core.hpp>
#include <boost/bind.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "image_point_reader.hpp"
#include "image_point_writer.hpp"
#include "iterator_reader.hpp"
#include "vector_writer.hpp"
#include "distortion.hpp"
#include "camera_properties_reader.hpp"
#include "read_image.hpp"
#include "util.hpp"

struct Undistort {
  cv::Mat K;
  cv::Mat K_inv; // to avoid inverting every iteration
  double w;

  cv::Point2d operator()(cv::Point2d x) const {
    // Undo intrinsics.
    cv::Mat X = imagePointToHomogeneous(x);
    X = K_inv * X;

    // Apply radial distortion.
    x = imagePointFromHomogeneous(X);
    x = undistort(x, w);

    // Redo intrinsics.
    X = imagePointToHomogeneous(x);
    X = K * X;

    return imagePointFromHomogeneous(X);
  }

  Undistort(const cv::Mat& K, double w) : K(), K_inv(), w(w) {
    this->K = K.clone();
    this->K_inv = K.inv();
  }
};

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Removes lens distortion from a set of points." << std::endl;
  usage << std::endl;
  usage << argv[0] << " distorted-points undistorted-points"
      " camera-properties" << std::endl;

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
  CHECK(ok) << "Could not load points";

  // Load camera properties.
  CameraProperties camera;
  CameraPropertiesReader camera_reader;
  ok = load(camera_file, camera, camera_reader);
  CHECK(ok) << "Could not load camera";

  // Build intrinsic matrix from camera.
  cv::Mat K = intrinsicMatrixFromCameraProperties(camera);

  // Undistort.
  std::transform(points.begin(), points.end(), points.begin(),
      Undistort(K, camera.distort_w));

  // Write out undistorted points.
  ImagePointWriter point_writer;
  ok = saveList(undistorted_points_file, points, point_writer);
  CHECK(ok) << "Could not save points";

  return 0;
}
