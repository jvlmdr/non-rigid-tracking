#include "videoseg/segmentation.hpp"
#include "videoseg/region.hpp"
#include <glog/logging.h>

namespace videoseg {

// Extracts a list of regions below a node.
struct GetDescendantRegions : public VisitRegion {
  typedef vector<const Rasterization*> RegionList;
  RegionList* regions;

  // Function to draw region.
  void operator()(VertexIndex vertex, const Rasterization& region) const {
    regions->push_back(&region);
  }

  GetDescendantRegions(RegionList& regions) : regions(&regions) {}
};

void getDescendantRegions(VertexIndex index,
                          const SegmentationTree& tree,
                          int t,
                          vector<const Rasterization*>& regions) {
  GetDescendantRegions get_descendants(regions);
  visitRegions(index, tree, t, get_descendants);
}

void flattenHierarchicalSegmentation(const SegmentationTree& tree,
                                     const map<VertexIndex, int>& leaves,
                                     int num_labels,
                                     int num_frames,
                                     VideoSegmentation& segmentation) {
  typedef vector<const Rasterization*> RegionList;

  for (int t = 0; t < num_frames; t += 1) {
    deque<RegionList> region_lists(num_labels);

    // Iterate through leaves to get list of regions belonging to each label.
    map<VertexIndex, int>::const_iterator leaf;
    for (leaf = leaves.begin(); leaf != leaves.end(); ++leaf) {
      VertexIndex index = leaf->first;
      int label = leaf->second;
      CHECK(0 <= label && label < num_labels) << "Label out of range";

      getDescendantRegions(index, tree, t, region_lists.at(label));
    }

    RepeatedPtrField<VideoSegmentation::Frame::Region> regions;
    int label = 0;

    // Iterate through regions.
    deque<RegionList>::const_iterator region_list;
    for (region_list = region_lists.begin();
        region_list != region_lists.end();
        ++region_list) {
      // Merge regions and add to list.
      Rasterization rasterization;
      mergeRegions(*region_list, rasterization);

      // Add region if non-empty.
      if (rasterization.scan_inter().size() > 0) {
        // Create a region with an ID.
        VideoSegmentation::Frame::Region region;
        region.set_id(label);
        region.mutable_raster()->Swap(&rasterization);
        // Add to list.
        regions.Add()->Swap(&region);
      }

      label += 1;
    }

    // Put list of regions into an image segmentation.
    VideoSegmentation::Frame frame;
    frame.mutable_regions()->Swap(&regions);

    // Add ImageSegmentation to VideoSegmentation.
    segmentation.mutable_frames()->Add()->Swap(&frame);
  }
}

}
