#include "videoseg/region.hpp"
#include <algorithm>
#include <glog/logging.h>

namespace videoseg {

struct IntervalBefore {
  inline bool operator()(const ScanInterval& lhs, const cv::Point& rhs) const {
    if (lhs.y() < rhs.y) {
      return true;
    } else if (rhs.y < lhs.y()) {
      return false;
    } else {
      if (lhs.right_x() < rhs.x) {
        return true;
      } else {
        return false;
      }
    }
  }

  inline bool operator()(const cv::Point& lhs, const ScanInterval& rhs) const {
    if (lhs.y < rhs.y()) {
      return true;
    } else if (rhs.y() < lhs.y) {
      return false;
    } else {
      if (lhs.x < rhs.left_x()) {
        return true;
      } else {
        return false;
      }
    }
  }
};

inline bool intervalBefore(const ScanInterval& lhs, const ScanInterval& rhs) {
  if (lhs.y() < rhs.y()) {
    return true;
  } else if (rhs.y() < lhs.y()) {
    return false;
  } else {
    if (lhs.left_x() < rhs.left_x()) {
      return true;
    } else if (rhs.left_x() < lhs.left_x()) {
      return false;
    } else {
      if (lhs.right_x() < rhs.right_x()) {
        return true;
      } else {
        return false;
      }
    }
  }
}

bool regionContains(const Rasterization& region, cv::Point pos) {
  typedef RepeatedPtrField<ScanInterval> IntervalList;
  const IntervalList& intervals = region.scan_inter();

  return std::binary_search(intervals.begin(), intervals.end(), pos,
      IntervalBefore());
}

////////////////////////////////////////////////////////////////////////////////

struct PositionInIntervalList {
  typedef RepeatedPtrField<ScanInterval> IntervalList;
  // Interval list and a position within that list.
  const IntervalList* intervals;
  IntervalList::const_iterator interval;

  // Ordered by ScanInterval ordering of element under cursor.
  bool operator<(const PositionInIntervalList& other) const {
    // Note order is reversed since STL implements a max heap not a min heap.
    return intervalBefore(*other.interval, *interval);
  }

  PositionInIntervalList(const IntervalList& intervals)
      : intervals(&intervals), interval(intervals.begin()) {}

  static PositionInIntervalList make(const Rasterization* region) {
    CHECK_GT(region->scan_inter().size(), 0) << "List must be non-empty";
    return PositionInIntervalList(region->scan_inter());
  }
};

void mergeRegions(const vector<const Rasterization*>& regions,
                  Rasterization& result) {
  typedef RepeatedPtrField<ScanInterval> ScanIntervalList;

  // Construct a priority queue of the regions.
  typedef vector<PositionInIntervalList> Heap;
  vector<PositionInIntervalList> heap;
  std::transform(regions.begin(), regions.end(), std::back_inserter(heap),
      PositionInIntervalList::make);
  std::make_heap(heap.begin(), heap.end());

  ScanIntervalList intervals;
  ScanInterval interval;
  bool first = true;

  // Merge scan intervals until heap is empty.
  while (!heap.empty()) {
    std::pop_heap(heap.begin(), heap.end());
    PositionInIntervalList cursor = heap.back();
    heap.pop_back();

    if (!first) {
      bool touch;

      if (interval.y() != cursor.interval->y()) {
        // Interval at cursor does not have same y coordinate.
        touch = false;
      } else {
        // Interval at cursor shares same y coordinate.
        if (interval.right_x() < cursor.interval->left_x() - 1) {
          // Previous interval ends before new interval begins.
          touch = false;
        } else {
          // Intervals touch.
          touch = true;
        }
      }

      if (touch) {
        // Merge the two intervals.
        interval.set_right_x(cursor.interval->right_x());
      } else {
        // Add old interval to list and start a new interval.
        intervals.Add()->Swap(&interval);
        interval = *cursor.interval;
      }
    } else {
      // Start a new interval.
      interval = *cursor.interval;
    }

    // Advance position in list.
    ++cursor.interval;

    if (cursor.interval != cursor.intervals->end()) {
      // More scan lines remain for this region, add cursor back into heap.
      heap.push_back(cursor);
      std::push_heap(heap.begin(), heap.end());
    } else {
      // Reached end of this cursor.
      // Check if it was the last region remaining.
      if (heap.empty()) {
        // Add last interval to the list.
        intervals.Add()->Swap(&interval);
      }
    }

    first = false;
  }

  result.mutable_scan_inter()->Swap(&intervals);
}

}
