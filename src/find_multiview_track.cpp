#include <vector>
#include <string>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/tools/roots.hpp>

#include "track.hpp"
#include "track_list.hpp"
#include "multiview_track_list.hpp"
#include "camera.hpp"
#include "distortion.hpp"
#include "util.hpp"
#include "quantize_ray.hpp"
#include "viterbi.hpp"

#include "track_list_reader.hpp"
#include "image_point_reader.hpp"
#include "read_lines.hpp"
#include "camera_pose_reader.hpp"
#include "camera_properties_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "image_point_writer.hpp"

DEFINE_double(delta, 1, "Minimum resolution (in pixels) per quantization");

std::pair<int, cv::Point2d> calibrateIndexedPoint(
    const std::pair<int, cv::Point2d>& point,
    const cv::Mat& K_inv) {
  cv::Point2d x = point.second;
  cv::Mat X = imagePointToHomogeneous(x);
  cv::Mat Y = K_inv * X;
  cv::Point2d y = imagePointFromHomogeneous(Y);
  return std::make_pair(point.first, y);
}

bool indexedPointIsNotUndistortable(const std::pair<int, cv::Point2d>& point,
                                    double w) {
  return !isUndistortable(point.second, w);
}

std::pair<int, cv::Point2d> undistortIndexedPoint(
    const std::pair<int, cv::Point2d>& point,
    double w) {
  return std::make_pair(point.first, undistort(point.second, w));
}

Track<cv::Point2d> calibrateAndUndistortTrack(
    const Track<cv::Point2d>& track,
    const CameraProperties& intrinsics) {
  // Calibrate each point, undo intrinsics.
  Track<cv::Point2d> calibrated;
  cv::Mat K_inv(intrinsics.matrix().inv());
  std::transform(track.begin(), track.end(),
      std::inserter(calibrated, calibrated.begin()),
      boost::bind(calibrateIndexedPoint, _1, K_inv));

  // Remove non-undistortable points.
  Track<cv::Point2d> valid;
  std::remove_copy_if(calibrated.begin(), calibrated.end(),
      std::inserter(valid, valid.begin()),
      boost::bind(indexedPointIsNotUndistortable, _1, intrinsics.distort_w));
  DLOG(INFO) << valid.size() << " / " << calibrated.size() <<
      " could be undistorted";

  // Undistort undistortable points.
  Track<cv::Point2d> undistorted;
  std::transform(valid.begin(), valid.end(),
    std::inserter(undistorted, undistorted.begin()),
    boost::bind(undistortIndexedPoint, _1, intrinsics.distort_w));

  return undistorted;
}

////////////////////////////////////////////////////////////////////////////////

void findMultiviewTrack(const Track<cv::Point2d>& track,
                        const std::vector<Camera>& cameras,
                        int selected,
                        MultiviewTrack<cv::Point2d>& multiview_track,
                        double lambda1,
                        double lambda2) {
  int num_views = cameras.size();

  multiview_track = MultiviewTrack<cv::Point2d>(num_views);

  LOG(INFO) << "Quantizing 3D ray";

  // For dynamic program, need to find the extent of the 3D ray in each frame.
  typedef std::map<double, cv::Point3d> Ray;
  std::vector<Ray> rays;
  {
    Track<cv::Point2d>::const_iterator point;
    for (point = track.begin(); point != track.end(); ++point) {
      Ray ray;
      quantizeRay(point->second, cameras, selected, FLAGS_delta, ray);
      rays.push_back(Ray());
      rays.back().swap(ray);
    }
  }

  LOG(INFO) << "Constructing Viterbi problem";

  // Construct unary costs.
  std::deque<std::vector<double> > unary_costs;
  {
    std::vector<Ray>::const_iterator ray;
    for (ray = rays.begin(); ray != rays.end(); ++ray) {
      std::vector<double> costs;

      Ray::const_iterator point;
      for (point = ray->begin(); point != ray->end(); ++point) {
        double cost = 0;

        std::vector<Camera>::const_iterator camera;
        for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
          // Compute cost.
          cost += 0;
        }
        costs.push_back(cost);
      }

      unary_costs.push_back(std::vector<double>());
      unary_costs.back().swap(costs);
    }
  }

  // Construct binary costs.
  std::vector<cv::Mat> binary_costs;
  if (rays.size() > 1) {
    std::vector<Ray>::const_iterator ray2 = rays.begin();
    std::vector<Ray>::const_iterator ray1;

    for (ray1 = ray2++; ray2 != rays.end(); ray1 = ray2++) {
      int n = ray1->size();
      int m = ray2->size();

      cv::Mat costs = cv::Mat_<double>(m, n, 0.);

      // Compute cost of moving from each position to another.
      int index1 = 0;
      Ray::const_iterator point1;
      for (point1 = ray1->begin(); point1 != ray1->end(); ++point1) {
        cv::Point3d x1 = point1->second;

        int index2 = 0;
        Ray::const_iterator point2;
        for (point2 = ray2->begin(); point2 != ray2->end(); ++point2) {
          cv::Point3d x2 = point2->second;

          // Cost of moving from x1 to x2 in 3D.
          double cost = lambda2 * cv::norm(x1 - x2);

          std::vector<Camera>::const_iterator camera;
          for (camera = cameras.begin(); camera != cameras.end(); ++camera) {
            // Compute appearance cost.
            cost += 0;
          }

          costs.at<double>(index2, index1) = cost;
          index2 += 1;
        }
        index1 += 1;
      }

      binary_costs.push_back(costs);
    }
  }

  LOG(INFO) << "Solving dynamic program";
  std::vector<int> solution;
  solveViterbi(unary_costs, binary_costs, solution);
}

void findMultiviewTracks(
    const TrackList<cv::Point2d>& tracks,
    const std::vector<Camera>& cameras,
    int selected,
    MultiviewTrackList<cv::Point2d>& multiview_tracks,
    int lambda1,
    int lambda2) {
  int num_views = cameras.size();
  multiview_tracks = MultiviewTrackList<cv::Point2d>(num_views);

  TrackList<cv::Point2d>::const_iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Find match for this track.
    MultiviewTrack<cv::Point2d> multiview_track;
    findMultiviewTrack(*track, cameras, selected, multiview_track, lambda1,
        lambda2);

    // Swap into end of list.
    multiview_tracks.push_back(MultiviewTrack<cv::Point2d>());
    multiview_tracks.back().swap(multiview_track);
  }
}

////////////////////////////////////////////////////////////////////////////////

std::string makeViewFilename(const std::string& format,
                             const std::string& name) {
  return boost::str(boost::format(format) % name);
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Finds an optimal multiview track given a track in one view" <<
      std::endl;
  usage << std::endl;
  usage << argv[0] << " view-index tracks image-format extrinsics-format "
      "intrinsics-format views num-frames multiview-tracks" << std::endl;
  usage << std::endl;
  usage << "Parameters:" << std::endl;
  usage << "view-index -- Zero-based index of view to which the original "
      "tracks belong" << std::endl;
  usage << "image-format -- e.g. images/%s/%07d.png" << std::endl;
  usage << "extrinsics-format -- e.g. extrinsics/%s.yaml" << std::endl;
  usage << "intrinsics-format -- e.g. intrinsics/%s.yaml" << std::endl;
  usage << "view -- Text file whose lines are the view names" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 9) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  bool ok;

  init(argc, argv);
  int main_view = boost::lexical_cast<int>(argv[1]);
  std::string input_tracks_file = argv[2];
  std::string image_format = argv[3];
  std::string extrinsics_format = argv[4];
  std::string intrinsics_format = argv[5];
  std::string views_file = argv[6];
  int num_frames = boost::lexical_cast<int>(argv[7]);
  std::string multiview_tracks_file = argv[8];

  // Load tracks.
  TrackList<cv::Point2d> input_tracks;
  ImagePointReader<double> point_reader;
  ok = loadTrackList(input_tracks_file, input_tracks, point_reader);
  CHECK(ok) << "Could not load tracks";
  LOG(INFO) << "Loaded " << input_tracks.size() << " single-view tracks";

  // Load names of views.
  std::vector<std::string> view_names;
  ok = readLines(views_file, view_names);
  CHECK(ok) << "Could not load view names";
  int num_views = view_names.size();
  LOG(INFO) << "Matching to " << num_views << " views";

  CHECK(main_view >= 0);
  CHECK(main_view < num_views);

  // Load properties of each view.
  std::vector<Camera> cameras;

  CameraPoseReader extrinsics_reader;
  CameraPropertiesReader intrinsics_reader;

  for (int view = 0; view < num_views; view += 1) {
    const std::string& name = view_names[view];

    // Load cameras for all views.
    CameraProperties intrinsics;
    std::string intrinsics_file = makeViewFilename(intrinsics_format, name);
    ok = load(intrinsics_file, intrinsics, intrinsics_reader);
    CHECK(ok) << "Could not load intrinsics";

    CameraPose extrinsics;
    std::string extrinsics_file = makeViewFilename(extrinsics_format, name);
    ok = load(extrinsics_file, extrinsics, extrinsics_reader);
    CHECK(ok) << "Could not load extrinsics";

    cameras.push_back(Camera(intrinsics, extrinsics));
  }

  // Find multiview tracks.
  MultiviewTrackList<cv::Point2d> multiview_tracks;
  findMultiviewTracks(input_tracks, cameras, main_view, multiview_tracks, 1, 1);

  // Save points and tracks out.
  ImagePointWriter<double> point_writer;
  ok = saveMultiviewTrackList(multiview_tracks_file, multiview_tracks,
      point_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
