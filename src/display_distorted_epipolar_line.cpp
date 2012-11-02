#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <cstdlib>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "distorted_epipolar_lines.hpp"
#include "camera_properties.hpp"
#include "sift_position.hpp"
#include "distortion.hpp"
#include "util.hpp"

#include "read_image.hpp"
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

int main(int argc, char** argv) {
  init(argc, argv);

  std::string keypoints_file = argv[1];
  std::string camera_file1 = argv[2];
  std::string camera_file2 = argv[3];
  std::string fund_mat_file = argv[4];
  std::string image_file1 = argv[5];
  std::string image_file2 = argv[6];

  bool ok;

  // Load camera properties.
  CameraProperties camera1;
  CameraProperties camera2;
  CameraPropertiesReader camera_reader;
  ok = load(camera_file1, camera1, camera_reader);
  CHECK(ok) << "Could not load intrinsics for first camera";
  ok = load(camera_file2, camera2, camera_reader);
  CHECK(ok) << "Could not load intrinsics for second camera";

  // Load fundamental matrix.
  cv::Mat F;
  MatrixReader matrix_reader;
  ok = load(fund_mat_file, F, matrix_reader);
  CHECK(ok) << "Could not load fundamental matrix";

  // Load keypoints.
  std::vector<SiftPosition> keypoints;
  SiftPositionReader keypoint_reader;
  ok = loadList(keypoints_file, keypoints, keypoint_reader);
  CHECK(ok) << "Could not load keypoints";

  // Shuffle keypoints.
  std::random_shuffle(keypoints.begin(), keypoints.end());

  // Load images.
  cv::Mat image1;
  cv::Mat image2;
  ok = readColorImage(image_file1, image1);
  CHECK(ok) << "Could not load first image";
  ok = readColorImage(image_file2, image2);
  CHECK(ok) << "Could not load second image";

  DistortedEpipolarRasterizer line_finder(camera2, F);
  line_finder.init();

  cv::Mat K1(camera1.matrix());
  cv::Mat K1_inv = K1.inv();

  std::vector<SiftPosition>::const_iterator keypoint;
  for (keypoint = keypoints.begin(); keypoint != keypoints.end(); ++keypoint) {
    cv::Point2d y1(keypoint->x, keypoint->y);
    // Undo intrinsics, undistort, and re-apply intrinsics.
    cv::Point2d x1 = imagePointFromHomogeneous(
        K1_inv * imagePointToHomogeneous(y1));
    x1 = undistort(x1, camera1.distort_w);
    x1 = imagePointFromHomogeneous(K1 * imagePointToHomogeneous(x1));

    // Find line.
    DistortedEpipolarRasterizer::PixelSet line;
    line_finder.compute(x1, line);

    cv::Mat display1 = image1.clone();
    cv::Mat display2 = image2.clone();

    // Show the point in the first image.
    cv::circle(display1, y1, 4, cv::Scalar(0, 0, 255), 2);
    cv::circle(display1, y1, 16, cv::Scalar(0, 0, 255), 2);
    cv::circle(display1, y1, 64, cv::Scalar(0, 0, 255), 2);

    // Show the line in the second image.
    DistortedEpipolarRasterizer::PixelSet::const_iterator pixel;
    for (pixel = line.begin(); pixel != line.end(); ++pixel) {
      cv::circle(display2, *pixel, 2, cv::Scalar(0, 0, 255), -1);
    }

    cv::pyrDown(display1, display1);
    cv::pyrDown(display2, display2);
    cv::imshow("Image 1", display1);
    cv::imshow("Image 2", display2);
    cv::waitKey();
  }

  return 0;
}
