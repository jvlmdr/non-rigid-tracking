#include <vector>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/bind.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "image_point_reader.hpp"
#include "image_point_writer.hpp"
#include "iterator_reader.hpp"
#include "vector_writer.hpp"
#include "camera_properties_reader.hpp"
#include "distortion.hpp"
#include "read_image.hpp"
#include "util.hpp"

DEFINE_string(output_file, "undistorted.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show matches?");

struct Distort {
  cv::Mat K;
  cv::Mat K_inv; // to avoid inverting every iteration
  double w;

  cv::Point2d operator()(cv::Point2d x) const {
    // Undo intrinsics.
    cv::Mat X = imagePointToHomogeneous(x);
    X = K_inv * X;

    // Apply radial distortion.
    x = imagePointFromHomogeneous(X);
    x = distort(x, w);

    // Redo intrinsics.
    X = imagePointToHomogeneous(x);
    X = K * X;

    return imagePointFromHomogeneous(X);
  }

  Distort(const cv::Mat& K, double w) : K(), K_inv(), w(w) {
    this->K = K.clone();
    this->K_inv = K.inv();
  }
};

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Removes lens distortion from an image." << std::endl;
  usage << std::endl;
  usage << argv[0] << " distorted-image camera-properties" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
}

int main(int argc, char** argv) {
  init(argc, argv);

  // Read required parameters.
  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }
  std::string image_file = argv[1];
  std::string camera_file = argv[2];

  bool ok;

  // Load image.
  cv::Mat image;
  cv::Mat gray_image;
  ok = readImage(image_file, image, gray_image);
  CHECK(ok) << "Could not read image";

  // Load camera properties.
  CameraProperties camera;
  CameraPropertiesReader camera_reader;
  ok = load(camera_file, camera, camera_reader);
  CHECK(ok) << "Could not load camera properties";

  // Build intrinsic matrix from camera.
  cv::Mat K = intrinsicMatrixFromCameraProperties(camera);

  int width = camera.image_size.width;
  int height = camera.image_size.height;

  // Construct a list of every pixel in the output image.
  std::vector<cv::Point2f> points;
  for (int i = 0; i < height; i += 1) {
    for (int j = 0; j < width; j += 1) {
      points.push_back(cv::Point2d(j, i));
    }
  }

  // Apply forward transform to every pixel in the output image.
  std::transform(points.begin(), points.end(), points.begin(),
      Distort(K, camera.distort_w));

  // Not sure about this....
  cv::Vec2f* data = static_cast<cv::Vec2f*>(
      static_cast<void*>(&points.front()));
  cv::Mat map = cv::Mat_<cv::Vec2f>(height, width, data);

  // Sample input image at these locations.
  cv::Mat undistorted;
  cv::remap(image, undistorted, map, cv::Mat(), cv::INTER_LINEAR);

  if (FLAGS_save) {
    cv::imwrite(FLAGS_output_file, undistorted);
  }

  if (FLAGS_display) {
    cv::imshow("undistorted", undistorted);
    cv::waitKey();
  }

  return 0;
}
