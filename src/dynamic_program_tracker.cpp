#include "dynamic_program_tracker.hpp"
#include <glog/logging.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "viterbi.hpp"

DynamicProgramTracker::DynamicProgramTracker(double lambda,
                                             int radius,
                                             bool fix_seed)
    : video_(NULL), lambda_(lambda), radius_(radius), fix_seed_(fix_seed) {}

DynamicProgramTracker::~DynamicProgramTracker() {}

void DynamicProgramTracker::init(const Video& video) {
  video_ = &video;
}

bool DynamicProgramTracker::track(const SpaceTimeImagePoint& point,
                                  Track<cv::Point2d>& track) const {
  cv::Mat initial_image;
  video_->get(point.t, initial_image);
  cv::Size size = initial_image.size();

  // Sample template from image.
  cv::Mat templ;
  cv::Point corner(point.x() - radius_, point.y() - radius_);
  int diameter = 2 * radius_ + 1;
  cv::Rect region(corner, cv::Size(diameter, diameter));
  templ = initial_image(region).clone();

  // Region of image in which template can be matched.
  cv::Rect interior(cv::Point(radius_, radius_),
      cv::Size(size.width - diameter + 1, size.height - diameter + 1));

  // Match the template to every image.
  std::vector<cv::Mat> appearance_costs;
  int n = video_->length();

  LOG(INFO) << "Performing cross-correlation";
  cv::Mat image;
  for (int t = 0; t < n; t += 1) {
    cv::Mat response;

    if (fix_seed_ && t == point.t) {
      // Set all other positions to +inf.
      response = cv::Mat_<double>(interior.size(),
          std::numeric_limits<double>::infinity());
      cv::Point center = cv::Point(std::floor(point.x() + 0.5),
                                   std::floor(point.y() + 0.5));
      cv::Point corner = center - cv::Point(radius_, radius_);
      response.at<double>(corner) = 0; // any finite constant
    } else {
      cv::Mat image;
      bool ok = video_->get(t, image);
      if (!ok) {
        return false;
      }
      // Evaluate response to template.
      cv::matchTemplate(image, templ, response, cv::TM_CCORR_NORMED);
      // Convert to 64-bit.
      response = cv::Mat_<double>(response);
      // Negate (minimizing not maximizing) and normalize.
      response = -1. / lambda_ * response;
    }

    // Add to list.
    appearance_costs.push_back(response);
  }

  LOG(INFO) << "Solving dynamic program";
  std::vector<cv::Vec2i> x;
  solveViterbiQuadratic2D(appearance_costs, x);

  // Convert to a track.
  track.clear();
  for (int t = 0; t < n; t += 1) {
    track[t] = cv::Point2d(x[t][1] + radius_, x[t][0] + radius_);
  }

  return true;
}
