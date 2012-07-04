#include "klt_tracker.hpp"
#include <stdexcept>

KltTracker::KltTracker()
    : tracks_(), active_(), frame_number_(0), tc_(), fl_() {}

void KltTracker::init(int max_num_features,
                      int min_clearance,
                      int window_size,
                      int min_eigenvalue,
                      int cornerness_jump,
                      double min_determinant,
                      int max_iterations,
                      double min_displacement,
                      double max_residual) {
  tc_ = KLTCreateTrackingContext();
  tc_->mindist = min_clearance;
  tc_->window_width = window_size;
  tc_->window_height = window_size;
  tc_->min_eigenvalue = min_eigenvalue;
  tc_->min_determinant = min_determinant;
  tc_->min_displacement = min_displacement;
  tc_->max_iterations = max_iterations;
  tc_->max_residue = max_residual;
  tc_->nSkippedPixels = cornerness_jump;

  fl_ = KLTCreateFeatureList(max_num_features);
}

// Accesses the tracks.
const TrackList& KltTracker::tracks() const {
  return tracks_;
}

int KltTracker::numFrames() const {
  return frame_number_;
}

// Returns the tracks which are currently active.
void KltTracker::activeTracks(std::vector<const Track*>& tracks) const {
  tracks.clear();
  for (Subset::const_iterator it = active_.begin(); it != active_.end(); ++it) {
    tracks.push_back(&tracks_[*it]);
  }
}

void KltTracker::feed(const cv::Mat& image) {
  if (image.type() != cv::DataType<uint8_t>::type) {
    throw std::runtime_error("image must be 8-bit 1-channel");
  }
  if (!image.isContinuous()) {
    throw std::runtime_error("image must be continuous");
  }

  if (frame_number_ == 0) {
    // First frame, initialise some features.
    KLTSelectGoodFeatures(tc_, const_cast<uint8_t*>(image.ptr<uint8_t>()),
        image.cols, image.rows, fl_);

    // Read out features.
    for (int i = 0; i < fl_->nFeatures; i += 1) {
      // Create a new track.
      tracks_.push_back(Track());

      // Insert first point in track.
      Track& track = tracks_.back();
      double x = fl_->feature[i]->x;
      double y = fl_->feature[i]->y;
      track[frame_number_] = cv::Point2d(x, y);

      // Set up as an active track.
      active_.push_back(i);
    }
  } else {
    // Track features from previous frame to next one.
    KLTTrackFeatures(tc_,
        const_cast<uint8_t *>(previous_.ptr<uint8_t>()),
        const_cast<uint8_t*>(image.ptr<uint8_t>()),
        image.cols, image.rows, fl_);

    // Update tracks.
    Subset::iterator index = active_.begin();
    while (index != active_.end()) {
      if (fl_->feature[*index]->val == KLT_TRACKED) {
        // Succesfully tracked, append to track.
        Track& track = tracks_[*index];
        double x = fl_->feature[*index]->x;
        double y = fl_->feature[*index]->y;
        track[frame_number_] = cv::Point2d(x, y);

        // Advance active track counter.
        ++index;
      } else {
        // Track was lost. Remove from active list.
        index = active_.erase(index);
      }
    }

    // Replace lost features.
    //KLTReplaceLostFeatures(tc, image, image.cols, image.rows, fl_);
  }

  image.copyTo(previous_);
  frame_number_ += 1;
}

bool KltTracker::write(cv::FileStorage& out) const {
  out << "[";

  for (Subset::const_iterator it = active_.begin(); it != active_.end(); ++it) {
    // Get track.
    const Track& track = tracks_[*it];
    // Get last point in track.
    const cv::Point2d& point = track.rbegin()->second;

    out << "{:" << "id" << *it << "x" << point.x << "y" << point.y << "}";
  }

  out << "]";

  return true;
}
