#ifndef VIDEOSEG_SEGMENTATION_HPP_
#define VIDEOSEG_SEGMENTATION_HPP_

#include "videoseg/using.hpp"
#include "videoseg/segmentation.pb.h"
#include "videoseg/tree.hpp"

namespace videoseg {

void flattenHierarchicalSegmentation(const SegmentationTree& tree,
                                     const map<VertexIndex, int>& leaves,
                                     int num_labels,
                                     int num_frames,
                                     VideoSegmentation& flat);

}

#endif
