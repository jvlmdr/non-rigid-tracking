#ifndef VIDEOSEG_SEGMENTATION_HPP_
#define VIDEOSEG_SEGMENTATION_HPP_

#include "videoseg/using.hpp"
#include "videoseg/segmentation.pb.h"
#include "videoseg/tree.hpp"

namespace videoseg {

struct RegionBefore {
  inline bool operator()(const VideoSegmentation::Frame::Region& lhs,
                         int rhs) const;
  inline bool operator()(int lhs,
                         const VideoSegmentation::Frame::Region& rhs) const;
};

inline bool regionBefore(const VideoSegmentation::Frame::Region& lhs,
                         const VideoSegmentation::Frame::Region& rhs);

void flattenHierarchicalSegmentation(const SegmentationTree& tree,
                                     const map<VertexIndex, int>& leaves,
                                     int num_labels,
                                     int num_frames,
                                     VideoSegmentation& flat);

}

#include "videoseg/segmentation.inl"

#endif
