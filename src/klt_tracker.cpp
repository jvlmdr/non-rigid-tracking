#include "klt_tracker.hpp"
#include <stdexcept>

KltTracker::KltTracker()
    : tracks_(), active_(), frame_number_(0), tc_(), fl_() {}

void KltTracker::init(const std::vector<cv::Point2d>& features,
                      int window_size,
                      double min_determinant,
                      int max_iterations,
                      double min_displacement,
                      double max_residual,
                      int consistency_mode) {
  tc_ = KLTCreateTrackingContext();

  tc_->window_width = window_size;
  tc_->window_height = window_size;
  tc_->min_determinant = min_determinant;
  tc_->min_displacement = min_displacement;
  tc_->max_iterations = max_iterations;
  tc_->max_residue = max_residual;

  tc_->affineConsistencyCheck = consistency_mode;

  tc_->sequentialMode = true;

  // Allocate enough for all features.
  fl_ = KLTCreateFeatureList(features.size());

  // Copy in to feature list.
  // This is a bit hacky since KLT doesn't give you a way to do it.
  for (int i = 0; i < int(features.size()); i += 1) {
    fl_->feature[i]->x = features[i].x;
    fl_->feature[i]->y = features[i].y;
    // A positive 'val' means the feature was just found.
    fl_->feature[i]->val = 1;

    // Other properties which are initialized in SelectGoodFeatures.
    fl_->feature[i]->aff_img = NULL;
    fl_->feature[i]->aff_img_gradx = NULL;
    fl_->feature[i]->aff_img_grady = NULL;
    fl_->feature[i]->aff_x = -1.0;
    fl_->feature[i]->aff_y = -1.0;
    fl_->feature[i]->aff_Axx = 1.0;
    fl_->feature[i]->aff_Ayx = 0.0;
    fl_->feature[i]->aff_Axy = 0.0;
    fl_->feature[i]->aff_Ayy = 1.0;
  }
}

// Accesses the tracks.
const TrackList_<cv::Point2d>& KltTracker::tracks() const {
  return tracks_;
}

int KltTracker::numFrames() const {
  return frame_number_;
}

// Returns the tracks which are currently active.
void KltTracker::activeTracks(
    std::vector<const Track_<cv::Point2d>*>& tracks) const {
  tracks.clear();

  Subset::const_iterator it;
  for (it = active_.begin(); it != active_.end(); ++it) {
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
    // Read out features.
    for (int i = 0; i < fl_->nFeatures; i += 1) {
      // Create a new track.
      tracks_.push_back(Track_<cv::Point2d>());

      // Insert first point in track.
      Track_<cv::Point2d>& track = tracks_.back();
      double x = fl_->feature[i]->x;
      double y = fl_->feature[i]->y;
      track[frame_number_] = cv::Point2d(x, y);

      // Set up as an active track.
      active_.push_back(i);
    }

    // Only need the previous image once, after that stored internally.
    image.copyTo(previous_);
  } else {
    // Track features from previous frame to next one.
    KLTTrackFeatures(tc_,
        const_cast<uint8_t *>(previous_.ptr<uint8_t>()),
        const_cast<uint8_t*>(image.ptr<uint8_t>()),
        image.cols, image.rows, fl_);

    // Only need previous image once.
    previous_ = cv::Mat();

    // Update tracks.
    Subset::iterator index = active_.begin();
    while (index != active_.end()) {
      if (fl_->feature[*index]->val == KLT_TRACKED) {
        // Succesfully tracked, append to track.
        Track_<cv::Point2d>& track = tracks_[*index];
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

  frame_number_ += 1;
}

bool KltTracker::write(cv::FileStorage& out) const {
  out << "[";

  for (Subset::const_iterator it = active_.begin(); it != active_.end(); ++it) {
    // Get track.
    const Track_<cv::Point2d>& track = tracks_[*it];
    // Get last point in track.
    const cv::Point2d& point = track.rbegin()->second;

    out << "{:" << "id" << *it << "x" << point.x << "y" << point.y << "}";
  }

  out << "]";

  return true;
}
