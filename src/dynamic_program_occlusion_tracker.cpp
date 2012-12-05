#include "dynamic_program_occlusion_tracker.hpp"
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include "viterbi.hpp"

DynamicProgramOcclusionTracker::DynamicProgramOcclusionTracker(
    double lambda,
    double penalty,
    int radius) : video_(NULL),
                  lambda_(lambda),
                  penalty_(penalty),
                  radius_(radius) {}

DynamicProgramOcclusionTracker::~DynamicProgramOcclusionTracker() {}

void DynamicProgramOcclusionTracker::init(const Video& video) {
  video_ = &video;
}

bool DynamicProgramOcclusionTracker::track(const SpaceTimeImagePoint& point,
                                           Track<cv::Point2d>& track) const {
  cv::Point center = cv::Point(std::floor(point.x() + 0.5),
                               std::floor(point.y() + 0.5));

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

  for (int t = 0; t < n; t += 1) {
    cv::Mat response;

    if (t == point.t) {
      // Set all other positions to +inf.
      response = cv::Mat_<double>(interior.size(),
          std::numeric_limits<double>::infinity());
      cv::Point corner = center - cv::Point(radius_, radius_);
      response.at<double>(corner) = 0; // any finite constant
    } else {
      cv::Mat image;
      bool ok = video_->get(t, image);
      CHECK(ok) << "Could not read image";
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

  // Cost of occlusion is uniform.
  cv::Mat occlusion_cost = cv::Mat_<double>(interior.size(),
      penalty_ / lambda_);
  std::vector<cv::Mat> occlusion_costs;
  for (int t = 0; t < n; t += 1) {
    occlusion_costs.push_back(occlusion_cost);
  }
  // Except in initial frame, when it is infinite.
  occlusion_costs[point.t] = cv::Mat();
  occlusion_costs[point.t] = cv::Mat_<double>(interior.size(),
      std::numeric_limits<double>::infinity());

  LOG(INFO) << "Solving dynamic program";
  std::vector<SplitVariable> solution;
  solveViterbiSplitQuadratic2D(appearance_costs, occlusion_costs, solution);

  // Convert to a track.
  track.clear();
  for (int t = 0; t < n; t += 1) {
    if (solution[t].set == 0) {
      track[t] = cv::Point2d(solution[t].index[1] + radius_,
          solution[t].index[0] + radius_);
    }
  }

  return true;
}
