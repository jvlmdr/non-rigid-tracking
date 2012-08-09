#include <iostream>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "camera_pose.hpp"
#include "camera_properties.hpp"
#include "reader.hpp"
#include "camera_pose_reader.hpp"
#include "camera_properties_reader.hpp"
#include "geometry.hpp"
#include "matrix_writer.hpp"

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Computes the fundamental matrix relating two cameras." << std::endl;
  usage << std::endl;
  usage << argv[0] << " intrinsics1 extrinsics1 intrinsics2 extrinsics2"
    " fund-mat" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
}

int main(int argc, char** argv) {
  init(argc, argv);

  // Read in mandatory parameters.
  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }
  std::string intrinsics_file1 = argv[1];
  std::string extrinsics_file1 = argv[2];
  std::string intrinsics_file2 = argv[3];
  std::string extrinsics_file2 = argv[4];
  std::string fund_mat_file = argv[5];

  bool ok;

  // Load intrinsics.
  CameraProperties properties1;
  CameraProperties properties2;

  CameraPropertiesReader properties_reader;
  ok = load(intrinsics_file1, properties1, properties_reader);
  CHECK(ok) << "Could not load intrinsic calibration";
  ok = load(intrinsics_file2, properties2, properties_reader);
  CHECK(ok) << "Could not load intrinsic calibration";

  // Load extrinsics.
  CameraPose pose1;
  CameraPose pose2;

  CameraPoseReader pose_reader;
  ok = load(extrinsics_file1, pose1, pose_reader);
  CHECK(ok) << "Could not load extrinsic calibration";
  ok = load(extrinsics_file2, pose2, pose_reader);
  CHECK(ok) << "Could not load extrinsic calibration";

  // Get calibrated projection matrices.
  cv::Mat P1 = projectionMatrixFromCameraPose(pose1);
  cv::Mat P2 = projectionMatrixFromCameraPose(pose2);

  // Get uncalibrated projection matrices.
  cv::Mat K1 = intrinsicMatrixFromCameraProperties(properties1);
  cv::Mat K2 = intrinsicMatrixFromCameraProperties(properties2);

  P1 = K1 * P1;
  P2 = K2 * P2;

  cv::Mat F = computeFundMatFromCameras(P1, P2);

  MatrixWriter matrix_writer;
  ok = save(fund_mat_file, F, matrix_writer);
  CHECK(ok) << "Could not save fundamental matrix";

  return 0;
}
