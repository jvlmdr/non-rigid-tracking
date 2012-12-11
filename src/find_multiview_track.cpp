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

#include "track_list_reader.hpp"
#include "image_point_reader.hpp"
#include "read_lines.hpp"
#include "camera_pose_reader.hpp"
#include "camera_properties_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "image_point_writer.hpp"

struct OtherView {
  int index;
  Camera camera;
};

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

  // Undistort undistortable points.
  Track<cv::Point2d> undistorted;
  std::transform(valid.begin(), valid.end(),
    std::inserter(undistorted, undistorted.begin()),
    boost::bind(undistortIndexedPoint, _1, intrinsics.distort_w));

  return undistorted;
}

////////////////////////////////////////////////////////////////////////////////

Camera extractCameraFromOtherView(const OtherView& view) {
  return view.camera;
}

double errorInDistanceFromPoint(cv::Point2d y,
                                double lambda,
                                const cv::Mat A,
                                const cv::Mat B,
                                double delta,
                                const CameraProperties& intrinsics) {
  cv::Mat X = A + lambda * B;
  cv::Point2d x = imagePointFromHomogeneous(X);
  x = intrinsics.distortAndUncalibrate(x);
  return cv::norm(x - y) - delta;
}

bool toleranceIsOk(double x, double y, double epsilon) {
  return std::abs(x - y) < epsilon;
}

void findExtentOfRay(const cv::Point2d& projection,
                     const Camera& camera,
                     const std::vector<Camera>& others) {
  // Apply inverse intrinsics.
  cv::Mat W = imagePointToHomogeneous(projection);
  cv::Mat K(camera.intrinsics().matrix());
  W = K.inv() * W;
  cv::Point2d w = imagePointFromHomogeneous(W);

  // Undistort point.
  w = undistort(w, camera.intrinsics().distort_w);

  // Camera center.
  cv::Point3d c(camera.extrinsics().center);

  // Find vector in nullspace of linear projection system, A = R_xy - w R_z.
  cv::Mat R(camera.extrinsics().rotation);
  cv::Mat A = R.rowRange(0, 2) - cv::Mat(w) * R.rowRange(2, 3);
  // 1D nullspace found trivially by cross-product.
  // Take negative i x j because z < 0 is in front of camera.
  cv::Mat V = -A.row(0).t().cross(A.row(1).t());
  cv::Point3d v(V.at<double>(0, 0), V.at<double>(0, 1), V.at<double>(0, 2));

  // Space of solutions parametrized by 3D line c + lambda v, with lambda >= 0.

  // For each image, find intersections of the projected line with the outer
  // radius of the lens distortion.
  std::vector<Camera>::const_iterator other;
  for (other = others.begin(); other != others.end(); ++other) {
    cv::Mat P(other->extrinsics().matrix());

    // Assume that the 3D line
    //   c + lambda v
    // defines a 2D line
    //   alpha a + (1 - alpha) b
    // with
    //   a = P([c 1]), b = P([v 0]).
    // Note that b is the projection of a point at infinity.
    cv::Mat C = worldPointToHomogeneous(c);
    cv::Mat V = worldPointToHomogeneous(v, 0);
    cv::Mat A = P * C;
    cv::Mat B = P * V;

    // Assume that line has a vanishing point (b is not at infinity).
    // TODO: Cope with non-vanishing-point case (b at infinity).
    cv::Point2d a = imagePointFromHomogeneous(A);
    CHECK(B.at<double>(2, 0) != 0);
    cv::Point2d b = imagePointFromHomogeneous(B);

    double a3 = A.at<double>(2, 0);
    double b3 = B.at<double>(2, 0);

    if (a3 > 0 && b3 > 0) {
      // Entire ray is behind camera.
      LOG(INFO) << "Ray is not observed";
      continue;
    }

    double lambda_max;
    if (b3 < 0) {
      // Ray ends in front of camera.
      double delta = 1;

      double w = other->intrinsics().distort_w;
      double k = distortRadius(cv::norm(v), w) / cv::norm(v);
      double a1 = A.at<double>(0, 0);
      double a2 = A.at<double>(1, 0);
      double b1 = B.at<double>(0, 0);
      double b2 = B.at<double>(1, 0);
      cv::Point2d d = cv::Point2d(a1, a2) - a3 / b3 * cv::Point2d(b1, b2);
      d.x *= other->intrinsics().focal_x;
      d.y *= other->intrinsics().focal_y;

      // Solve a quadratic system.
      double aa = sqr(delta) * sqr(b3);
      double bb = sqr(delta) * 2 * a3 * b3;
      double cc = sqr(delta) * sqr(a3) - sqr(k) * d.dot(d);

      double discriminant = sqr(bb) - 4 * aa * cc;
      CHECK(discriminant >= 0) << "Cannot solve system";

      double lambda1 = (-bb - std::sqrt(discriminant)) / (2 * aa);
      double lambda2 = (-bb + std::sqrt(discriminant)) / (2 * aa);
      CHECK(lambda2 > 0) << "No positive solutions";
      if (discriminant > 0) {
        CHECK(lambda1 < 0) << "Two positive solutions";
      }

      lambda_max = lambda2;
      LOG(INFO) << "lambda_max => " << lambda_max;

      // Now find next lambda.
      double lambda = lambda_max;
      cv::Mat X = A + lambda * B;
      cv::Point2d x = imagePointFromHomogeneous(X);
      x = other->intrinsics().distortAndUncalibrate(x);
      LOG(INFO) << "x(lambda_max) => " << x;

      cv::Point2d center = other->intrinsics().distortAndUncalibrate(a);
      cv::Point2d vanishing_point = other->intrinsics().distortAndUncalibrate(b);
      LOG(INFO) << "image of center => " << center;
      LOG(INFO) << "image of vanishing point => " << vanishing_point;

      {
        cv::Mat X = A + 0 * B;
        cv::Point2d x = imagePointFromHomogeneous(X);
        x = other->intrinsics().distortAndUncalibrate(x);
        LOG(INFO) << "x(0) => " << x;
      }

      bool converged = false;

      while (!converged) {
        double lambda_prime = lambda;
        cv::Point2d x_prime = x;

        double f_max = errorInDistanceFromPoint(x_prime, 0, A, B, delta,
              other->intrinsics());
        LOG(INFO) << "f_max => " << f_max;
        if (f_max < 0) {
          // No solution is possible.
          LOG(INFO) << "Converged (f_max < 0)";
          converged = true;
        }

        if (!converged) {
          std::pair<double, double> interval = boost::math::tools::bisect(
                boost::bind(errorInDistanceFromPoint, x_prime, _1, A, B, delta,
                  other->intrinsics()),
                0., lambda_prime,
                boost::math::tools::eps_tolerance<double>(32));
          lambda = interval.second;
          LOG(INFO) << "lambda => " << lambda;

          // Guard against limit cycles.
          CHECK(lambda != lambda_prime) << "Entered limit cycle";

          X = A + lambda * B;
          x = imagePointFromHomogeneous(X);
          x = other->intrinsics().distortAndUncalibrate(x);
          LOG(INFO) << "x(lambda) => " << x;
          LOG(INFO) << "image of center => " << center;
        }
      }
    } else {
      // Ray ends behind camera, find intersection of ray with edge of image.
      LOG(INFO) << "No vanishing point";
      lambda_max = 0;
    }

    /*
    double max_lambda;
    // If vanishing point is inside circle, no bound on lambda.
    if (circle.contains(b)) {
      LOG(INFO) << "Vanishing point is visible";
      // Bound lambda by minimum which gives sub-pixel accuracy?
      // Nah, do this later.
      max_lambda = std::numeric_limits<double>::infinity();
    } else {
      // If vanishing point is outside circle, find intersection.
      max_lambda = circle.findIntersection(a, b);
    }
    */
  }
}

void findMultiviewTrack(const Track<cv::Point2d>& track,
                        const Camera& camera,
                        const std::vector<OtherView>& other_views,
                        MultiviewTrack<cv::Point2d>& multiview_track) {
  int num_other_views = other_views.size();
  int num_views = num_other_views + 1;

  multiview_track = MultiviewTrack<cv::Point2d>(num_views);

  std::vector<Camera> other_cameras;
  std::transform(other_views.begin(), other_views.end(),
      std::back_inserter(other_cameras), extractCameraFromOtherView);

  // For dynamic program, need to find the extent of the 3D ray in each frame.
  Track<cv::Point2d>::const_iterator point;
  for (point = track.begin(); point != track.end(); ++point) {
    findExtentOfRay(point->second, camera, other_cameras);
  }
}

void findMultiviewTracks(
    const TrackList<cv::Point2d>& tracks,
    const Camera& camera,
    const std::vector<OtherView>& other_views,
    MultiviewTrackList<cv::Point2d>& multiview_tracks) {
  int num_views = other_views.size() + 1;
  multiview_tracks = MultiviewTrackList<cv::Point2d>(num_views);

  TrackList<cv::Point2d>::const_iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Find match for this track.
    MultiviewTrack<cv::Point2d> multiview_track;
    findMultiviewTrack(*track, camera, other_views, multiview_track);

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
  std::vector<OtherView> other_views;
  Camera camera;

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

    Camera view_camera(intrinsics, extrinsics);

    if (view == main_view) {
      // Set intrinsics of main view.
      camera = view_camera;
    } else {
      OtherView other_view;
      other_view.index = view;
      other_view.camera = view_camera;
      other_views.push_back(other_view);
    }
  }

  // Undistort points in original view.
  TrackList<cv::Point2d> undistorted_tracks;
  std::transform(input_tracks.begin(), input_tracks.end(),
      std::back_inserter(undistorted_tracks),
      boost::bind(calibrateAndUndistortTrack, _1, camera.intrinsics()));

  // Find multiview tracks.
  MultiviewTrackList<cv::Point2d> multiview_tracks;
  findMultiviewTracks(undistorted_tracks, camera, other_views,
      multiview_tracks);

  // Save points and tracks out.
  ImagePointWriter<double> point_writer;
  ok = saveMultiviewTrackList(multiview_tracks_file, multiview_tracks,
      point_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
