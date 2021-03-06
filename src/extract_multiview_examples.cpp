#include <numeric>
#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "sift_feature.hpp"
#include "track.hpp"
#include "track_list.hpp"
#include "util.hpp"
#include "distorted_epipolar_lines.hpp"
#include "distortion.hpp"
#include "extract_sift.hpp"

#include "read_lines.hpp"
#include "read_image.hpp"
#include "track_list_reader.hpp"
#include "sift_feature_reader.hpp"
#include "iterator_reader.hpp"
#include "camera_properties_reader.hpp"
#include "matrix_reader.hpp"

#include "iterator_writer.hpp"
#include "classifier_writer.hpp"

const int NUM_OCTAVE_LAYERS = 3;
const double SIGMA = 1.6;

std::string makeImageFilename(const std::string& format,
                              const std::string& view,
                              int time) {
  return boost::str(boost::format(format) % view % (time + 1));
}

std::string makeViewFilename(const std::string& format,
                             const std::string& view) {
  return boost::str(boost::format(format) % view);
}

std::string makeViewPairFilename(const std::string& format,
                                 const std::string& view1,
                                 const std::string& view2) {
  return boost::str(boost::format(format) % view1 % view2);
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Extracts examples using tracking and epipolar matches" << std::endl;
  usage << std::endl;
  usage << argv[0] << " descriptor-tracks image-format fund-mat-format "
      "intrinsics-format views view-index frame-index examples" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 9) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

void extractFeaturesAlongLine(
    const std::vector<cv::Point>& line,
    const cv::Mat& image,
    const std::vector<double> scales,
    const std::vector<double> angles,
    std::deque<std::deque<SiftFeature> >& line_features) {
  line_features.clear();

  SiftExtractor sift(image, NUM_OCTAVE_LAYERS, SIGMA);

  // Extract features along the line at different scales and orientations.
  std::vector<cv::Point>::const_iterator point;
  for (point = line.begin(); point != line.end(); ++point) {
    std::deque<SiftFeature> point_features;

    std::vector<double>::const_iterator scale;
    for (scale = scales.begin(); scale != scales.end(); ++scale) {
      std::vector<double>::const_iterator angle;
      for (angle = angles.begin(); angle != angles.end(); ++angle) {
        // Extract descriptor.
        SiftPosition position(point->x, point->y, *scale, *angle);
        Descriptor descriptor;
        sift.extractDescriptor(position, descriptor);

        SiftFeature feature;
        feature.position = position;
        feature.descriptor.swap(descriptor);

        point_features.push_back(SiftFeature());
        point_features.back().swap(feature);
      }
    }

    line_features.push_back(std::deque<SiftFeature>());
    line_features.back().swap(point_features);
  }
}

void extractExamplesForView(const TrackList<SiftFeature>& tracks,
                            const cv::Mat& F,
                            const CameraProperties& camera1,
                            const CameraProperties& camera2,
                            const std::string& image_format,
                            const std::string& view,
                            int time,
                            const std::vector<double>& scales,
                            const std::vector<double>& angles) {
  // March along epipolar line.
  DistortedEpipolarRasterizer rasterizer(camera2, F);
  rasterizer.init();

  cv::Mat K1(camera1.matrix());
  cv::Mat K1_inv = K1.inv();

  TrackList<SiftFeature>::const_iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    Track<SiftFeature>::const_iterator iter = track->find(time);
    CHECK(iter != track->end()) << "Track does not contain the current frame";

    const SiftFeature& feature = iter->second;
    cv::Point2d y1(feature.position.x, feature.position.y);

    // Undo intrinsics, undistort, and re-apply intrinsics.
    cv::Point2d x1 = affineTransformImagePoint(y1, K1_inv);
    x1 = undistort(x1, camera1.distort_w);
    x1 = affineTransformImagePoint(x1, K1);

    // Extract the pixels of the epipolar line.
    std::vector<cv::Point> line;
    rasterizer.compute(x1, line);

    // Load first image.
    cv::Mat image;
    std::string image_file = makeImageFilename(image_format, view, time);
    bool ok = readGrayImage(image_file, image);
    CHECK(ok) << "Could not load image";

    std::deque<std::deque<SiftFeature> > features;
    extractFeaturesAlongLine(line, image, scales, angles, features);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string descriptor_tracks_file = argv[1];
  std::string image_format = argv[2];
  std::string fund_mat_format = argv[3];
  std::string intrinsics_format = argv[4];
  std::string views_file = argv[5];
  int view1 = boost::lexical_cast<int>(argv[6]);
  int time = boost::lexical_cast<int>(argv[7]);
  std::string examples_file = argv[8];

  // Load descriptors tracks.
  TrackList<SiftFeature> features;
  SiftFeatureReader feature_reader;
  bool ok = loadTrackList(descriptor_tracks_file, features, feature_reader);
  CHECK(ok) << "Could not load descriptor tracks";
  LOG(INFO) << "Loaded " << features.size() << " features";

  // Load names of views.
  std::vector<std::string> view_names;
  ok = readLines(views_file, view_names);
  CHECK(ok) << "Could not load view names";
  int num_views = view_names.size();

  // Load intrinsics for main camera.
  CameraProperties camera1;
  std::string camera_file1 = makeViewFilename(intrinsics_format,
      view_names[view1]);
  CameraPropertiesReader camera_reader;
  ok = load(camera_file1, camera1, camera_reader);
  CHECK(ok) << "Could not load intrinsics for main camera";

  // For each view.
  for (int view2 = 0; view2 < num_views; view2 += 1) {
    if (view2 != view1) {
      // Load fundamental matrix.
      cv::Mat F;
      int i = view1;
      int j = view2;
      bool swap = false;
      if (view2 < view1) {
        std::swap(i, j);
        swap = true;
      }
      std::string fund_mat_file = makeViewPairFilename(fund_mat_format,
          view_names[i], view_names[j]);
      MatrixReader matrix_reader;
      ok = load(fund_mat_file, F, matrix_reader);
      CHECK(ok) << "Could not load fundamental matrix";
      if (swap) {
        F = F.t();
      }

      // Load camera properties.
      CameraProperties camera2;
      std::string camera_file2 = makeViewFilename(intrinsics_format,
          view_names[view2]);
      ok = load(camera_file2, camera2, camera_reader);
      CHECK(ok) << "Could not load intrinsics for second camera";

      std::vector<double> scales;
      scales.push_back(4);
      scales.push_back(8);
      scales.push_back(16);
      scales.push_back(32);
      scales.push_back(64);

      std::vector<double> angles;
      angles.push_back(0 * M_PI / 4.);
      angles.push_back(1 * M_PI / 4.);
      angles.push_back(2 * M_PI / 4.);
      angles.push_back(3 * M_PI / 4.);
      angles.push_back(4 * M_PI / 4.);
      angles.push_back(5 * M_PI / 4.);
      angles.push_back(6 * M_PI / 4.);
      angles.push_back(7 * M_PI / 4.);

      extractExamplesForView(features, F, camera1, camera2, image_format,
          view_names[view2], time, scales, angles);
    }
  }

  return 0;
}
