#include "videoseg/draw-region.hpp"
#include <glog/logging.h>

namespace videoseg {

void fillRegion(const Rasterization& region, cv::Mat& image, cv::Vec3b color) {
  typedef RepeatedPtrField<ScanInterval> IntervalList;
  const IntervalList& intervals = region.scan_inter();
  IntervalList::const_iterator interval;

  for (interval = intervals.begin(); interval != intervals.end(); ++interval) {
    int y = interval->y();

    for (int i = interval->left_x(); i <= interval->right_x(); i += 1) {
      image.at<cv::Vec3b>(y, i) = color;
    }
  }
}

void drawRegionBoundary(const Rasterization& region,
                        cv::Mat& image,
                        cv::Vec3b color) {
  typedef RepeatedPtrField<ScanInterval> IntervalList;
  const IntervalList& intervals = region.scan_inter();

  if (intervals.size() == 0) {
    // Empty region.
    return;
  }

  cv::Size size = image.size();

  vector<bool> previous_row;
  vector<bool> row;

  IntervalList::const_iterator interval = intervals.begin();
  // Range for current row.
  IntervalList::const_iterator begin;
  IntervalList::const_iterator end = intervals.begin();

  bool finished = false;
  bool first_row = true;
  bool second_row = true;
  bool last_row = false;
  int previous_i = -1;
  int i = -1;

  while (!finished) {
    vector<bool> next_row;
    bool end_of_image = false;
    int next_i = -1;

    if (!last_row) {
      // Read next row.
      next_row.assign(size.width, false);
      bool end_of_row = false;
      next_i = interval->y();

      while (!end_of_image && !end_of_row) {
        // Populate next row.
        for (int j = interval->left_x(); j <= interval->right_x(); j += 1) {
          next_row[j] = true;
        }

        ++interval;

        if (interval == intervals.end()) {
          end_of_image = true;
        } else {
          if (interval->y() != next_i) {
            end_of_row = true;
          }
        }
      }
    }

    // Render current row.
    if (!first_row) {
      IntervalList::const_iterator it;
      for (it = begin; it != end; ++it) {
        // Set colors.
        for (int j = it->left_x(); j <= it->right_x(); j += 1) {
          bool boundary = false;

          if (j == it->left_x()) {
            boundary = true;
          } else if (j == it->right_x()) {
            boundary = true;
          } else if (second_row) {
            boundary = true;
          } else if (last_row) {
            boundary = true;
          } else {
            CHECK_EQ(previous_row.size(), size.width);
            CHECK_EQ(next_row.size(), size.width);

            if (previous_i != i - 1) {
              boundary = true;
            } else if (!previous_row[j]) {
              boundary = true;
            } else if (next_i != i + 1) {
              boundary = true;
            } else if (!next_row[j]) {
              boundary = true;
            }
          }

          if (boundary) {
            image.at<cv::Vec3b>(i, j) = color;
          }
        }
      }
    }

    // Shuffle previous, current and next.
    previous_row.swap(row);
    row.swap(next_row);
    begin = end;
    end = interval;
    previous_i = i;
    i = next_i;

    if (!first_row) {
      second_row = false;
    }
    first_row = false;

    if (last_row) {
      finished = true;
    }
    if (end_of_image) {
      last_row = true;
    }
  }
}

void copyRegion(const Rasterization& region,
                const cv::Mat& src,
                cv::Mat& dst) {
  typedef RepeatedPtrField<ScanInterval> IntervalList;
  const IntervalList& intervals = region.scan_inter();
  IntervalList::const_iterator interval;

  for (interval = intervals.begin(); interval != intervals.end(); ++interval) {
    int y = interval->y();
    for (int i = interval->left_x(); i <= interval->right_x(); i += 1) {
      dst.at<cv::Vec3b>(y, i) = src.at<cv::Vec3b>(y, i);
    }
  }
}

void copyAndBlendRegion(const Rasterization& region,
                        const cv::Mat& src,
                        cv::Vec3b& color,
                        double alpha,
                        cv::Mat& dst) {
  typedef RepeatedPtrField<ScanInterval> IntervalList;
  const IntervalList& intervals = region.scan_inter();
  IntervalList::const_iterator interval;

  alpha = std::max(alpha, 0.);
  alpha = std::min(alpha, 1.);

  for (interval = intervals.begin(); interval != intervals.end(); ++interval) {
    int y = interval->y();
    for (int i = interval->left_x(); i <= interval->right_x(); i += 1) {
      cv::Vec3b pixel = src.at<cv::Vec3b>(y, i);
      dst.at<cv::Vec3b>(y, i) = (1. - alpha) * pixel + alpha * color;
    }
  }
}

} // namespace videoseg
