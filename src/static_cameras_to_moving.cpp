#include <sstream>
#include <cstdlib>
#include <string>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/lexical_cast.hpp>

#include "camera.hpp"

#include "camera_properties_reader.hpp"
#include "camera_pose_reader.hpp"

#include "track_writer.hpp"
#include "camera_writer.hpp"

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Converts a static camera to a \"moving\" camera." << std::endl;
  usage << std::endl;
  usage << argv[0] << " intrinsics extrinsics dynamic-camera num-frames" <<
      std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 5) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string intrinsics_file = argv[1];
  std::string extrinsics_file = argv[2];
  std::string dynamic_camera_file = argv[3];
  int num_frames = boost::lexical_cast<int>(argv[4]);
  CHECK(num_frames > 0) << "Number of frames must be positive";

  bool ok;

  // Load camera.
  CameraProperties intrinsics;
  CameraPropertiesReader properties_reader;
  ok = load(intrinsics_file, intrinsics, properties_reader);
  CHECK(ok) << "Could not load intrinsic calibration";

  CameraPose extrinsics;
  CameraPoseReader pose_reader;
  ok = load(extrinsics_file, extrinsics, pose_reader);
  CHECK(ok) << "Could not load extrinsic calibration";

  // Construct camera.
  Camera camera(intrinsics, extrinsics);

  // Copy into every frame.
  Track<Camera> cameras;
  for (int t = 0; t < num_frames; t += 1) {
    cameras[t] = camera;
  }

  CameraWriter camera_writer;
  TrackWriter<Camera> track_writer(camera_writer);
  ok = save(dynamic_camera_file, cameras, track_writer);
  CHECK(ok) << "Could not save cameras";

  return 0;
}
