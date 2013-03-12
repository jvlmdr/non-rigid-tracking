#include "tracking/using.hpp"
#include <sstream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "tracking/track-list.hpp"
#include "tracking/warp.hpp"
#include "tracking/flow.hpp"
#include "tracking/similarity-warp.hpp"
#include "tracking/translation-warp.hpp"
#include "util/sqr.hpp"
#include "util/random-color.hpp"
#include <boost/format.hpp>

using namespace tracking;

DEFINE_bool(display, true, "Show tracking in window");
DEFINE_string(save, "", "Directory to save frames to, ignored if empty");

DEFINE_int32(radius, 8, "Half of [patch size - 1]");
DEFINE_double(threshold, 1e-3, "Cornerness threshold");
DEFINE_double(min_clearance, 8., "Minimum clearance before re-detecting");
DEFINE_double(mask_sigma, 4., "Sigma to use in mask");
DEFINE_double(max_residual, 0.05,
    "Maximum relative average intensity difference");

DEFINE_bool(check_condition, true,
    "Check condition of Jacobian during tracking?");
DEFINE_double(max_condition, 1e3,
    "Maximum condition of Jacobian during tracking");
DEFINE_int32(max_iter, 100,
    "Maximum number of iterations in non-linear problem");
DEFINE_bool(fatal_max_iter, true,
    "Does reaching the iteration limit without converging terminate the "
    "track?");

const double FUNCTION_TOLERANCE = 1e-6;
const double GRADIENT_TOLERANCE = 1e-6;
const double PARAMETER_TOLERANCE = 1e-6;

// Visualization parameters.
const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const int LINE_THICKNESS = 1;

string makeFilename(const string& format, int n) {
  return boost::str(boost::format(format) % n);
}

// Describes a feature which is currently being tracked.
class TrackedFeature {
  public:
    TrackedFeature() : similarity_(), translation_(), appearance_(), color_() {}

    TrackedFeature(const TranslationWarp& warp, const cv::Vec3b& color)
      : similarity_(),
        translation_(new TranslationWarp(warp)),
        appearance_(),
        color_(color) {}

    TrackedFeature(const SimilarityWarp& warp, const cv::Vec3b& color)
      : similarity_(new SimilarityWarp(warp)),
        translation_(),
        appearance_(),
        color_(color) {}

    TrackedFeature(const TrackedFeature& other)
        : similarity_(),
          translation_(),
          appearance_(other.appearance_.clone()),
          color_(other.color_) {
      if (other.similarity_) {
        similarity_.reset(new SimilarityWarp(*other.similarity_));
      }
      if (other.translation_) {
        translation_.reset(new TranslationWarp(*other.translation_));
      }
    }

    TrackedFeature& operator=(const TrackedFeature& other) {
      similarity_.reset();
      if (other.similarity_) {
        similarity_.reset(new SimilarityWarp(*other.similarity_));
      }

      translation_.reset();
      if (other.translation_) {
        translation_.reset(new TranslationWarp(*other.translation_));
      }

      appearance_ = other.appearance_.clone();
      color_ = other.color_;

      return *this;
    }

    inline const cv::Mat& appearance() const { return appearance_; }
    inline cv::Mat& appearance() { return appearance_; }
    inline cv::Vec3b color() const { return color_; }

    inline bool isSimilarity() const {
      return static_cast<bool>(similarity_);
    }

    inline bool isTranslation() const {
      return static_cast<bool>(translation_);
    }

    inline const Warp& warp() const {
      if (isSimilarity()) {
        return *similarity_;
      } else {
        return *translation_;
      }
    }

    inline Warp& warp() {
      if (isSimilarity()) {
        return *similarity_;
      } else {
        return *translation_;
      }
    }

    inline const SimilarityWarp& similarity() const {
      return *similarity_;
    }

    inline SimilarityWarp& similarity() {
      return *similarity_;
    }

    inline const TranslationWarp& translation() const {
      return *translation_;
    }

    inline TranslationWarp& translation() {
      return *translation_;
    }

    void swap(TrackedFeature& other) {
      similarity_.swap(other.similarity_);
      translation_.swap(other.translation_);
      std::swap(appearance_, other.appearance_);
      std::swap(color_, other.color_);
    }

  private:
    // Only one of these is non-null.
    scoped_ptr<SimilarityWarp> similarity_;
    scoped_ptr<TranslationWarp> translation_;

    cv::Mat appearance_;
    cv::Vec3b color_;
};

// Describes a list of features currently being tracked.
class TrackedFeatureList {
  public:
    TrackedFeatureList() : features_(), count_(0) {}

    typedef map<int, TrackedFeature>::const_iterator const_iterator;
    typedef map<int, TrackedFeature>::iterator iterator;

    inline const_iterator begin() const {
      return features_.begin();
    }

    inline const_iterator end() const {
      return features_.end();
    }

    inline iterator begin() {
      return features_.begin();
    }

    inline iterator end() {
      return features_.end();
    }

    int size() const {
      return features_.size();
    }

    TrackedFeature& add() {
      return (features_[count_++] = TrackedFeature());
    }

    void erase(iterator position) {
      features_.erase(position);
    }

    int erase(int id) {
      return features_.erase(id);
    }

  private:
    map<int, TrackedFeature> features_;
    int count_;
};

cv::Mat makeGaussian(double sigma, int width) {
  cv::Mat gaussian = cv::Mat_<double>(width, width);
  int radius = (width - 1) / 2;
  double sigma2 = sigma * sigma;

  for (int i = 0; i < width; i += 1) {
    double u = i - radius;
    for (int j = 0; j < width; j += 1) {
      double v = j - radius;
      double d2 = u * u + v * v;
      gaussian.at<double>(i, j) = std::exp(-d2 / (2. * sigma2));
    }
  }

  cv::Mat mask = cv::Mat_<double>::zeros(width, width);
  cv::Point center(radius, radius);
  cv::circle(mask, center, radius, 1., -1);
  cv::multiply(gaussian, mask, gaussian);

  return gaussian;
}

// Computes the average pixel residual, weighted by W.
double patchResidual(const cv::Mat& A, const cv::Mat& B, const cv::Mat& W) {
  // D = abs(A - B) .* W
  cv::Mat D = cv::abs(A - B);
  cv::multiply(D, W, D);

  double num = std::accumulate(D.begin<double>(), D.end<double>(), 0.);
  double denom = std::accumulate(W.begin<double>(), W.end<double>(), 0.);

  return num / denom;
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Automatically detects and tracks featuers." << std::endl;
  usage << std::endl;
  usage << argv[0] << " video tracks" << std::endl;
  usage << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

class OccupancyMap {
  public:
    OccupancyMap(cv::Size size, double radius)
        : occupied_(size, false), radius_(radius) {}

    inline bool occupied(cv::Point pos) const {
      return occupied_(pos);
    }

    void add(cv::Point pos) {
      // Set region to occupied.
      int min_y = std::floor(pos.y - radius_ + 0.5);
      int max_y = std::floor(pos.y + radius_ + 0.5);
      // Clip to bounds of image.
      min_y = std::max(min_y, 0);
      max_y = std::min(max_y, occupied_.rows - 1);

      for (int y = min_y; y <= max_y; y += 1) {
        double delta = std::sqrt(sqr(radius_) - sqr(y - pos.y));
        // Set region to occupied.
        int min_x = std::floor(pos.x - delta + 0.5);
        int max_x = std::floor(pos.x + delta + 0.5);
        // Clip to bounds of image.
        min_x = std::max(min_x, 0);
        max_x = std::min(max_x, occupied_.cols - 1);

        for (int x = min_x; x <= max_x; x += 1) {
          occupied_(cv::Point(x, y)) = true;
        }
      }
    }

  private:
    cv::Mat_<bool> occupied_;
    double radius_;
};

//class ScaleSpaceOccupancyMap {
//  public:
//    ScaleSpaceOccupancyMap(int width,
//                           int height,
//                           double radius,
//                           double step,
//                           double sigma)
//        : width_(width),
//          height_(height),
//          radius_(radius),
//          log_step_(std::log(step)),
//          sigma_(sigma) {
//      int num_levels = std::log(std::min(width_, height_)) / log_step_;
//
//      for (int level = 0; level < num_levels; level += 1) {
//        double scale = std::exp(level * log_step_);
//        int w = std::ceil(width / scale);
//        int h = std::ceil(height / scale);
//        occupied_.push_back(cv::Mat_<bool>(h, w, false));
//      }
//    }
//
//    inline bool occupied(double x, double y, double scale) const {
//      // Determine nearest level.
//      int level = std::floor(std::log(scale) / log_step_ + 0.5);
//      // Clamp to valid range.
//      level = std::max(level, 0);
//      level = std::min(level, int(occupied_.size()) - 1);
//      // Divide x and y by scale of the nearest level.
//      double level_scale = std::exp(level * log_step_);
//      x = x / level_scale;
//      y = y / level_scale;
//
//      int i = std::floor(y + 0.5);
//      int j = std::floor(x + 0.5);
//
//      return occupied_[level](i, j);
//    }
//
//    void add(double x, double y, double scale) {
//    }
//
//  private:
//    double width_;
//    double height_;
//    double radius_;
//    double log_step_;
//    double sigma_;
//    vector<cv::Mat_<bool> > occupied_;
//};

struct ScoredPixel {
  cv::Point pos;
  double score;

  bool operator<(const ScoredPixel& other) const {
    return score < other.score;
  }

  ScoredPixel(cv::Point pos, double score) : pos(pos), score(score) {}
};

// This is an object to avoid re-allocating cornerness map.
class FeatureDetector {
  public:
    FeatureDetector() : cornerness_() {}
    FeatureDetector(cv::Size size) : cornerness_(cv::Mat_<float>(size)) {}

    void reserve(cv::Size size) {
      cornerness_.create(size, cv::DataType<float>::type);
    }

    void detect(const cv::Mat& image,
                const cv::Mat& float_image,
                TrackedFeatureList& features,
                int block_size,
                int k_size,
                double min_clearance,
                double threshold,
                int diameter,
                int interpolation) {
      // Calculate cornerness at every pixel.
      cv::cornerMinEigenVal(float_image, cornerness_, block_size, k_size);

      // Populate occupancy map with location of existing translation features.
      const cv::Size size = cornerness_.size();
      OccupancyMap occupancy(size, min_clearance);

      TrackedFeatureList::const_iterator it;
      for (it = features.begin(); it != features.end(); ++it) {
        const TrackedFeature& feature = it->second;

        if (feature.isTranslation()) {
          const TranslationWarp& warp = feature.translation();
          occupancy.add(cv::Point2d(warp.x(), warp.y()));
        }
      }

      // Build list of local maxima.
      typedef vector<ScoredPixel> PixelList;
      PixelList pixels;
      int radius = (diameter - 1) / 2;

      for (int x = radius; x < size.width - radius - 1; x += 1) {
        for (int y = radius; y < size.height - radius - 1; y += 1) {
          cv::Point pos(x, y);
          double score = cornerness_.at<float>(pos);
          bool use;

          double right = cornerness_.at<float>(pos + cv::Point( 1,  0));
          double left  = cornerness_.at<float>(pos + cv::Point(-1,  0));
          double above = cornerness_.at<float>(pos + cv::Point( 0, -1));
          double below = cornerness_.at<float>(pos + cv::Point( 0,  1));

          if (score < threshold) {
            use = false;
          } else {
            if (score >= right && score >= left &&
                score >= above && score >= below) {
              use = true;
            } else {
              use = false;
            }
          }

          if (use) {
            pixels.push_back(ScoredPixel(pos, score));
          }
        }
      }

      std::make_heap(pixels.begin(), pixels.end());
      int num_added = 0;

      while (!pixels.empty()) {
        // Take next best pixel.
        const ScoredPixel& pixel = pixels.front();
        std::pop_heap(pixels.begin(), pixels.end());
        pixels.pop_back();

        // Add to list if not occupied.
        if (!occupancy.occupied(pixel.pos)) {
          TranslationWarp warp(pixel.pos.x, pixel.pos.y);
          // Sample patch appearance.
          cv::Mat patch;
          samplePatch(warp, image, patch, diameter, false, interpolation);

          TrackedFeature feature(warp, randomColor(SATURATION, BRIGHTNESS));
          std::swap(feature.appearance(), patch);

          features.add().swap(feature);
          num_added += 1;
        }

        // Add to occupancy map.
        occupancy.add(pixel.pos);
      }

      LOG(INFO) << "Added " << num_added << " features";
    }

  private:
    cv::Mat cornerness_;
};

void addFeaturesToFrame(const TrackedFeatureList& features,
                        TrackList::Frame& frame) {
  TrackedFeatureList::const_iterator it;

  for (it = features.begin(); it != features.end(); ++it) {
    int id = it->first;
    const TrackedFeature& feature = it->second;

    if (!feature.isTranslation()) {
      continue;
    }

    const TranslationWarp& warp = feature.translation();

    TrackList::Point point;
    point.set_id(id);
    point.set_x(warp.x());
    point.set_y(warp.y());

    frame.mutable_points()->Add()->Swap(&point);
  }
}

void drawFeatures(const TrackedFeatureList& features,
                  cv::Mat& image,
                  int radius) {
  // Draw features.
  TrackedFeatureList::const_iterator it;
  for (it = features.begin(); it != features.end(); ++it) {
    const TrackedFeature& feature = it->second;
    cv::Vec3b color = feature.color();
    cv::Scalar scalar(color[0], color[1], color[2]);
    feature.warp().draw(image, radius, scalar, LINE_THICKNESS);
  }
}

void detectAndTrack(cv::VideoCapture& capture,
                    TrackList& tracks,
                    int radius,
                    double threshold,
                    double min_clearance,
                    double mask_sigma,
                    double max_residual,
                    const FlowOptions& options,
                    bool display,
                    const std::string& save) {
  // Construct mask.
  int diameter = radius * 2 + 1;
  cv::Mat mask = makeGaussian(mask_sigma, diameter);
  // Draw tracks?
  bool render = (display || !save.empty());

  // Loop state.
  bool end = false;
  int n = 0;

  // Features currently being tracked.
  TrackedFeatureList features;

  // Memory that is re-used every loop.
  cv::Mat image;
  cv::Mat color_image;
  cv::Mat integer_image;
  cv::Mat float_image;
  cv::Mat ddx;
  cv::Mat ddy;
  cv::Mat visualization;

  const cv::Mat diff = (cv::Mat_<double>(1, 3) << -0.5, 0, 0.5);
  const cv::Mat identity = (cv::Mat_<double>(1, 1) << 1);

  FeatureDetector detector;

  // Read frames of video.
  while (!end) {
    // Read next frame.
    bool ok = capture.read(color_image);
    if (!ok) {
      // Reached end.
      end = true;
      continue;
    }

    LOG(INFO) << "Tracking " << features.size() << " features";

    // Convert color to intensity.
    cv::cvtColor(color_image, integer_image, CV_BGR2GRAY);
    // Convert to floating point in [0, 1].
    integer_image.convertTo(image, cv::DataType<double>::type, 1. / 255);
    // Purely for OpenCV corner detection.
    integer_image.convertTo(float_image, cv::DataType<float>::type, 1. / 255);
    // Compute gradient images once.
    cv::sepFilter2D(image, ddx, -1, diff, identity);
    cv::sepFilter2D(image, ddy, -1, identity, diff);

    int num_removed = 0;

    TrackedFeatureList::iterator it = features.begin();
    while (it != features.end()) {
      TrackedFeature& feature = it->second;

      // Track features from the previous image.
      bool tracked = trackPatch(feature.warp(), feature.appearance(), image,
          ddx, ddy, mask, options);

      if (tracked) {
        // Sample patch for appearance check and template update.
        cv::Mat patch;
        samplePatch(feature.warp(), image, patch, diameter, false,
            options.interpolation);

        // Do appearance check.
        double residual = patchResidual(patch, feature.appearance(), mask);
        if (residual > max_residual) {
          DLOG(INFO) << "Appearance residual too large (" << residual <<
              " > " << max_residual << ")";
          tracked = false;
        }

        // Update appearance.
        std::swap(feature.appearance(), patch);
      }

      // Erase feature if failed to track. Move to next feature.
      if (!tracked) {
        features.erase(it++);
        num_removed += 1;
      } else {
        ++it;
      }
    }

    LOG(INFO) << "Removed " << num_removed << " features";

    // Detect new features.
    detector.reserve(color_image.size());
    detector.detect(image, float_image, features, radius, 3, min_clearance,
        threshold, diameter, options.interpolation);

    // Add all features to structure.
    TrackList::Frame frame;
    addFeaturesToFrame(features, frame);
    tracks.mutable_frames()->Add()->Swap(&frame);

    if (render) {
      // Convert grayscale back to color for displaying.
      cv::cvtColor(integer_image, visualization, CV_GRAY2BGR);
      drawFeatures(features, visualization, radius);

      if (display) {
        cv::imshow("Tracks", visualization);
        cv::waitKey(1000. / 30);
      }
      if (!save.empty()) {
        string file = makeFilename(save, n);
        cv::imwrite(file, visualization);
      }
    }

    n += 1;
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  string video_file = argv[1];
  string tracks_file = argv[2];

  bool ok;

  // Open video stream.
  cv::VideoCapture capture;
  ok = capture.open(video_file);
  CHECK(ok) << "Could not open video stream";

  FlowOptions options;
  options.solver_options.linear_solver_type = ceres::DENSE_QR;
  options.solver_options.max_num_iterations = FLAGS_max_iter;
  options.solver_options.function_tolerance = FUNCTION_TOLERANCE;
  options.solver_options.gradient_tolerance = GRADIENT_TOLERANCE;
  options.solver_options.parameter_tolerance = PARAMETER_TOLERANCE;
  options.solver_options.logging_type = ceres::SILENT;
  options.check_condition = FLAGS_check_condition;
  options.max_condition = FLAGS_max_condition;
  options.iteration_limit_is_fatal = FLAGS_fatal_max_iter;
  options.interpolation = cv::INTER_LINEAR;

  TrackList tracks;
  detectAndTrack(capture, tracks, FLAGS_radius, FLAGS_threshold,
      FLAGS_min_clearance, FLAGS_mask_sigma, FLAGS_max_residual, options,
      FLAGS_display, FLAGS_save);

  LOG(INFO) << "Saving tracks...";
  std::ofstream ofs(tracks_file.c_str(), std::ios::trunc | std::ios::binary);
  ok = tracks.SerializeToOstream(&ofs);
  if (!ok) {
    LOG(FATAL) << "Could not save output";
  }

  return 0;
}
