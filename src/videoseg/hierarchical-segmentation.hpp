/*
 *  Created by Matthias Grundmann on 6/15/09.
 *  Copyright 2009 Matthias Grundmann. All rights reserved.
 *
 */

#ifndef VIDEOSEG_HIERARCHICAL_SEGMENTATION_HPP_
#define VIDEOSEG_HIERARCHICAL_SEGMENTATION_HPP_

#include "videoseg/using.hpp"
#include "videoseg/hierarchical-segmentation.pb.h"

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core_c.h>

namespace videoseg {

using boost::shared_ptr;
typedef SegmentationDesc::Region2D Region2D;
typedef SegmentationDesc::ShapeMoments ShapeMoments;
typedef SegmentationDesc::CompoundRegion CompoundRegion;
typedef SegmentationDesc::HierarchyLevel HierarchyLevel;
typedef RepeatedPtrField<HierarchyLevel> Hierarchy;

// Common usage for all functions:
// The level of the hierarchy is passed in hierarchy_level.
// 0:   denotes the over-segmentation.
// >1 : denotes a hierarchical level
//
// Functions will threshold the passed level to max. level present in the hierarchy.
//
// The pixel level segmentation in SegmentationDesc and the actual hierarchy can
// be separated, as the hierarchy is saved only ONCE for the whole video volume.

// Finds region in proto-buffer.
// Note: region_id is not necessarily index of region.
const Region2D& GetRegion2DFromId(int id,
                                  const SegmentationDesc& desc);

bool ContainsRegion2D(int id,
                      const SegmentationDesc& desc);

Region2D* GetMutableRegion2DFromId(int id,
                                   SegmentationDesc* desc);

const CompoundRegion& GetCompoundRegionFromId(int id,
                                              const HierarchyLevel& hierarchy);

CompoundRegion* GetMutableCompoundRegionFromId(int id,
                                               HierarchyLevel* hierarchy);

const SegmentationDesc::AggregatedDescriptor&
GetDescriptorFromId(int id, const SegmentationDesc& desc);

int GetParentId(int region_id,
                int level,
                int query_level,
                const Hierarchy& seg_hier);

void SortRegions2DById(SegmentationDesc* desc);
void SortCompoundRegionsById(SegmentationDesc* desc, int level);

// Obtains map, that maps a parent_id to list of over-segmented Regions.
// Specifically each region in seg will be inserted to into the vector
// specified by its parent's id at hierarchy level level.
typedef unordered_map<int, vector<const Region2D*> > ParentMap;
void GetParentMap(int level,
                  const SegmentationDesc& seg,
                  const Hierarchy& seg_hier,
                  ParentMap* parent_map);

// Returns list of ALL spatio-temporal children in the segmentation tree at query_level
// for the specified region at level.
// Note: If you use per-frame operations you probably want to use GetParentMap.
void GetChildrenIds(int region_id,
                    int level,
                    int query_level,
                    const Hierarchy& seg_hier,
                    vector<int>* children_ids);

struct ShapeDescriptor {
  float center_x;
  float center_y;
  // Magnitude of major and minor axis.
  float mag_major;
  float mag_minor;
  // Direction of major and minor axis.
  CvPoint2D32f dir_major;
  CvPoint2D32f dir_minor;
};

// Return value indicates if reasonable shape descriptor could be computed.
// For example if region is too small (< 5 pixels), major and minor axis
// can be unstable. However, computation of the centroid is always stable.
bool GetShapeDescriptorFromRegions(const vector<const Region2D*>& regions,
                                   ShapeDescriptor* shape_desc);

bool GetShapeDescriptorFromRegion(const Region2D& r,
                                  ShapeDescriptor* shape_desc);

// Lexicographic ordering for Intervals.
class ScanIntervalComparator {
 public:
  bool operator()(const ScanInterval& lhs, const ScanInterval& rhs) const {
    return lhs.y() < rhs.y() || (lhs.y() == rhs.y() && lhs.left_x () < rhs.left_x());
  }
};


// Perform binary search on scanlines.
inline RepeatedPtrField<ScanInterval>::const_iterator
LocateScanLine(int y, const Rasterization& raster) {
  ScanInterval scan_inter;
  scan_inter.set_y(y);
  scan_inter.set_left_x(-1);
  return lower_bound(raster.scan_inter().begin(),
                     raster.scan_inter().end(),
                     scan_inter,
                     ScanIntervalComparator());
}

void MergeRasterization(const Rasterization& raster_1,
                        const Rasterization& raster_2,
                        Rasterization* merged_raster);

// List of tuple (#frame, Rasterization). Orderded by first argument.
typedef vector< std::pair<int, shared_ptr<Rasterization> > > Rasterization3D;

void MergeRasterization3D(const Rasterization3D& raster_1,
                          const Rasterization3D& raster_2,
                          Rasterization3D* merged_raster);

struct Rasterization3DLocator {
  bool operator()(const std::pair<int, shared_ptr<Rasterization> >& raster_1,
                  const std::pair<int, shared_ptr<Rasterization> >& raster_2) {
    return raster_1.first <= raster_2.first;
  }
};

inline Rasterization3D::const_iterator LocateRasterization(
    int frame, const Rasterization3D& raster) {
  return lower_bound(raster.begin(),
                     raster.end(),
                     std::make_pair(frame, shared_ptr<Rasterization>()));
}

int RasterizationArea(const Rasterization& raster);

// If id is set, region_id will be printed instead of center cross.
void DrawShapeDescriptors(const vector<int>& overseg_region_ids,
                          const SegmentationDesc& desc,
                          char* img,
                          int width_step,
                          int width,
                          int height,
                          int channels,
                          const int* region_id = 0);

// Computes and initializes the shape descriptor of a specific region.
void SetShapeDescriptorFromRegion(Region2D* r);

// Removes spatio-temporal regions not completely contained in [lhs, rhs]
// from a hierarchy level. Note: Does not alter children or parent members.
void ConstrainHierarchyToFrameInterval(int lhs,
                                       int rhs,
                                       const HierarchyLevel& input_hierachy,
                                       HierarchyLevel* constraint_hierarchy);

// Converts Segmentation description to image by assigning each pixel its
// corresponding region id.
void SegmentationDescToIdImage(int* img,
                               int width_step,
                               int width,
                               int height,
                               int hierarchy_level,
                               const SegmentationDesc& seg,
                               const Hierarchy* seg_hier);

// Returns region_id at corresponding (x, y) location in image,
// return value -1 indicates error.
int GetOversegmentedRegionIdFromPoint(int x,
                                      int y,
                                      const SegmentationDesc& seg);

void BuildGlobalHierarchy(const Hierarchy& chunk_hierarchy,
                          int chunk_frame_number,
                          Hierarchy* global_hierarchy);

bool VerifyGlobalHierarchy(const Hierarchy& hierarchy);

// Returns number of connected components, using disjoint-set operations. If number
// is larger than zero, components contains rasterization of each component.
int ConnectedComponents(const Rasterization& raster,
                        vector<Rasterization>* components);

} // namespace videoseg

#endif
