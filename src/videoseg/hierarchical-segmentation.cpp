/*
 *  Created by Matthias Grundmann on 6/15/09.
 *  Copyright 2009 Matthias Grundmann. All rights reserved.
 *
 */

#include "videoseg/hierarchical-segmentation.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>

typedef unsigned char uchar;
#include <boost/pending/disjoint_sets.hpp>

#ifdef _WIN32
#undef min
#undef max
#endif

#define _USE_MATH_DEFINES
#include <math.h>

// Moved from imagefilter here to remove library dependency.
namespace {

template <class T>
T* PtrOffset(T* t, int offset) {
  return reinterpret_cast<T*>(reinterpret_cast<uchar*>(t) + offset);
}

template <class T>
const T* PtrOffset(const T* t, int offset) {
  return reinterpret_cast<const T*>(reinterpret_cast<const uchar*>(t) + offset);
}

inline CvPoint trunc(const CvPoint2D32f& pt) {
  return cvPoint((int)pt.x, (int)pt.y);
}

inline float dot(const CvPoint2D32f& lhs, const CvPoint2D32f& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

inline float sq_norm(const CvPoint2D32f& lhs) {
  return dot(lhs, lhs);
}

inline CvPoint2D32f operator*(const CvPoint2D32f& lhs, float f) {
  return cvPoint2D32f(lhs.x * f, lhs.y * f);
}

inline CvPoint2D32f normalize(const CvPoint2D32f& pt) {
  return pt * (1.0 / sqrt(sq_norm(pt)));
}

inline CvPoint2D32f operator+(const CvPoint2D32f& lhs, const CvPoint2D32f& rhs) {
  return cvPoint2D32f(lhs.x + rhs.x, lhs.y + rhs.y);
}

inline CvPoint2D32f operator-(const CvPoint2D32f& lhs, const CvPoint2D32f& rhs) {
  return cvPoint2D32f(lhs.x - rhs.x, lhs.y - rhs.y);
}

} // namespace

namespace videoseg {

namespace {

struct Region2DComparator {
  bool operator()(const Region2D& lhs, const Region2D& rhs) const {
    return lhs.id() < rhs.id();
  }
};

struct AggregatedDescriptorComparator {
  AggregatedDescriptorComparator(int id = 0) : id_(id) { }
  bool operator()(const SegmentationDesc::AggregatedDescriptor& lhs,
                  const SegmentationDesc::AggregatedDescriptor& rhs) const {
    return lhs.id() < rhs.id();
  }

 private:
  int id_;
};

struct CompoundRegionComparator {
  bool operator()(const CompoundRegion& lhs, const CompoundRegion& rhs) const {
    return lhs.id() < rhs.id();
  }
};

}  // namepace

const Region2D& GetRegion2DFromId(int id,
                                  const SegmentationDesc& desc) {
  Region2D to_find;
  to_find.set_id(id);
  RepeatedPtrField<Region2D>::const_iterator region =
      std::lower_bound(desc.region().begin(),
                       desc.region().end(),
                       to_find,
                       Region2DComparator());
  //ASSERT_LOG(region != desc.region().end() &&
  //           region->id() == id);

  return *region;
}

bool ContainsRegion2D(int id,
                      const SegmentationDesc& desc) {
  Region2D to_find;
  to_find.set_id(id);
  RepeatedPtrField<Region2D>::const_iterator region =
    std::lower_bound(desc.region().begin(),
                     desc.region().end(),
                     to_find,
                     Region2DComparator());
  return region != desc.region().end() && region->id() == id;
}

Region2D* GetMutableRegion2DFromId(int id,
                                   SegmentationDesc* desc) {
  Region2D to_find;
  to_find.set_id(id);
  RepeatedPtrField<Region2D>::iterator region =
  std::lower_bound(desc->mutable_region()->begin(),
                   desc->mutable_region()->end(),
                   to_find,
                   Region2DComparator());
  //ASSERT_LOG(region != desc->mutable_region()->end() &&
  //           region->id() == id);

  return &(*region);
}

const CompoundRegion& GetCompoundRegionFromId(int id,
                                              const HierarchyLevel& hier_level) {
  CompoundRegion to_find;
  to_find.set_id(id);

  RepeatedPtrField<CompoundRegion>::const_iterator region =
  std::lower_bound(hier_level.region().begin(),
                   hier_level.region().end(),
                   to_find,
                   CompoundRegionComparator());
  //ASSERT_LOG(region != hier_level.region().end() &&
  //           region->id() == id);

  return *region;
}

CompoundRegion* GetMutableCompoundRegionFromId(int id,
                                               HierarchyLevel* hier_level) {
  CompoundRegion to_find;
  to_find.set_id(id);

  RepeatedPtrField<CompoundRegion>::iterator region =
  std::lower_bound(hier_level->mutable_region()->begin(),
                   hier_level->mutable_region()->end(),
                   to_find,
                   CompoundRegionComparator());
  //ASSERT_LOG(region != hier_level->mutable_region()->end() &&
  //           region->id() == id);

  return &(*region);
}

const SegmentationDesc::AggregatedDescriptor&
GetDescriptorFromId(int id, const SegmentationDesc& desc) {
  SegmentationDesc::AggregatedDescriptor to_find;
  to_find.set_id(id);
  RepeatedPtrField<SegmentationDesc::AggregatedDescriptor>::const_iterator descriptor =
  std::lower_bound(desc.descriptors().begin(),
                   desc.descriptors().end(),
                   to_find,
                   AggregatedDescriptorComparator());
  //ASSURE_LOG(descriptor != desc.descriptors().end() &&
  //           descriptor->id() == id) << "Not found";

  return *descriptor;
}

int GetParentId(int region_id,
                int level,
                int query_level,
                const Hierarchy& seg_hier) {
  if (level == query_level) {
    return region_id;
  }

  //ASSERT_LOG(query_level > level);

  int parent_id;
  //ASSERT_LOG(seg_hier.size() >= query_level);
  parent_id = GetCompoundRegionFromId(region_id, seg_hier.Get(level)).parent_id();

  if (query_level == level + 1) {
    return parent_id;
  } else {
    return GetParentId(parent_id, level + 1, query_level, seg_hier);
  }
}

void SortRegions2DById(SegmentationDesc* desc) {
  std::sort(desc->mutable_region()->begin(),
            desc->mutable_region()->end(),
            Region2DComparator());
}

void SortCompoundRegionsById(SegmentationDesc* desc, int level) {
  //ASSURE_LOG(level >= 0);
  std::sort(desc->mutable_hierarchy(level)->mutable_region()->begin(),
            desc->mutable_hierarchy(level)->mutable_region()->end(),
            CompoundRegionComparator());
}

void GetParentMap(int level,
                  const SegmentationDesc& seg,
                  const Hierarchy& seg_hier,
                  unordered_map<int, vector<const Region2D*> >* parent_map) {
  if (level >= seg_hier.size()) {
    level = seg_hier.size() - 1;
    //LOG(WARNING) << "Clamping requested level to " << level;
  }

  // Fill each region.
  const RepeatedPtrField<Region2D>& regions = seg.region();
  for(RepeatedPtrField<Region2D>::const_iterator r = regions.begin();
      r != regions.end();
      ++r) {
    int parent_id = GetParentId(r->id(), 0, level, seg_hier);
    (*parent_map)[parent_id].push_back(&*r);
  }
}

void GetChildrenIds(int region_id,
                    int level,
                    int query_level,
                    const Hierarchy& seg_hier,
                    vector<int>* children_ids) {
  //ASSURE_LOG(level > query_level);
  const CompoundRegion& region = GetCompoundRegionFromId(region_id,
                                                         seg_hier.Get(level));

  if (query_level + 1 == level) {
    // Add all children and return.
    for (int i = 0; i < region.child_id_size(); ++i) {
      children_ids->push_back(region.child_id(i));
    }
  } else {
    for (int i = 0; i < region.child_id_size(); ++i) {
      GetChildrenIds(region.child_id(i),
                     level - 1,
                     query_level,
                     seg_hier,
                     children_ids);
    }
  }
}

bool GetShapeDescriptorFromRegions(const vector<const Region2D*>& regions,
                                   ShapeDescriptor* shape_desc) {
  // Compute mixed moments.
  float mixed_x = 0, mixed_y = 0, mixed_xx = 0, mixed_xy = 0, mixed_yy = 0;
  float area_sum = 0;

  for (vector<const Region2D*>::const_iterator region_ptr = regions.begin();
       region_ptr != regions.end();
       ++region_ptr) {
    const Region2D& region = **region_ptr;
    const float area = region.shape_moments().size();
    area_sum += area;

    mixed_x += region.shape_moments().mean_x() * area;
    mixed_y += region.shape_moments().mean_y() * area;
    mixed_xx += region.shape_moments().moment_xx() * area;
    mixed_xy += region.shape_moments().moment_xy() * area;
    mixed_yy += region.shape_moments().moment_yy() * area;
  }

  //ASSURE_LOG(area_sum > 0);

  // Normalize by inv_area_sum.
  const float inv_area_sum = 1.0f / area_sum;
  mixed_x *= inv_area_sum;
  mixed_y *= inv_area_sum;
  mixed_xx *= inv_area_sum;
  mixed_xy *= inv_area_sum;
  mixed_yy *= inv_area_sum;

  shape_desc->center_x = mixed_x;
  shape_desc->center_y = mixed_y;

  if (area_sum < 10) {
    return false;
  }

  // Compute variance matrix.
  const float var_xx = mixed_xx - mixed_x * mixed_x;
  const float var_xy = mixed_xy - mixed_x * mixed_y;
  const float var_yy = mixed_yy - mixed_y * mixed_y;

  // Compute eigenvectors of variance matrix.
  const float trace = var_xx + var_yy;
  const float det = var_xx * var_yy - var_xy * var_xy;

  const float discriminant = 0.25 * trace * trace - det;
  //ASSURE_LOG(discriminant >= 0);
  const float sqrt_disc = sqrt(discriminant);
  const float e_1 = trace * 0.5 - sqrt_disc;
  const float e_2 = trace * 0.5 + sqrt_disc;

  CvPoint2D32f ev_1, ev_2;
  if (fabs(var_xy) < 1e-6) {
    ev_1 = cvPoint2D32f(1, 0);
    ev_2 = cvPoint2D32f(0, 1);
  } else {
    ev_1 = normalize(cvPoint2D32f(e_1 - var_yy, var_xy));
    ev_2 = normalize(cvPoint2D32f(e_2 - var_yy, var_xy));
  }

  float e_1_scaled = sqrt(fabs(e_1));
  float e_2_scaled = sqrt(fabs(e_2));

  if (e_1_scaled < e_2_scaled) {
    std::swap(e_1_scaled, e_2_scaled);
    std::swap(ev_1, ev_2);
  }

  shape_desc->center_x = mixed_x;
  shape_desc->center_y = mixed_y;
  shape_desc->mag_major = e_1_scaled;
  shape_desc->mag_minor = e_2_scaled;
  shape_desc->dir_major = ev_1;
  shape_desc->dir_minor = ev_2;

  return true;
}

bool GetShapeDescriptorFromRegion(const Region2D& r,
                                  ShapeDescriptor* shape_desc) {
  return GetShapeDescriptorFromRegions(vector<const Region2D*>(1, &r), shape_desc);
}

void MergeRasterization(const Rasterization& lhs,
                        const Rasterization& rhs,
                        Rasterization* merged) {
  //ASSURE_LOG(merged);

  RepeatedPtrField<ScanInterval>::const_iterator lhs_scan = lhs.scan_inter().begin();
  RepeatedPtrField<ScanInterval>::const_iterator rhs_scan = rhs.scan_inter().begin();

  RepeatedPtrField<ScanInterval>::const_iterator lhs_end = lhs.scan_inter().end();
  RepeatedPtrField<ScanInterval>::const_iterator rhs_end = rhs.scan_inter().end();

  vector<int> interval_offsets;
  while (lhs_scan != lhs_end || rhs_scan != rhs_end) {
    const int lhs_y = (lhs_scan == lhs_end ? 1 << 30 : lhs_scan->y());
    const int rhs_y = (rhs_scan == rhs_end ? 1 << 30 : rhs_scan->y());

    if (lhs_y < rhs_y) {
      merged->add_scan_inter()->CopyFrom(*lhs_scan++);
    } else if (rhs_y < lhs_y) {
      merged->add_scan_inter()->CopyFrom(*rhs_scan++);
    } else {
      // y-coords are equal.
      //ASSERT_LOG(lhs_y == rhs_y);

      interval_offsets.clear();
      bool left_cond, right_cond;
      while ( (left_cond = (lhs_scan != lhs_end && lhs_scan->y() == lhs_y)) ||
              (right_cond = (rhs_scan != rhs_end && rhs_scan->y() == rhs_y)) ) {
        const int lhs_x = left_cond ? lhs_scan->left_x() : 1 << 30;
        const int rhs_x = right_cond ? rhs_scan->left_x() : 1 << 30;
        if (lhs_x < rhs_x) {
          interval_offsets.push_back(lhs_scan->left_x());
          interval_offsets.push_back(lhs_scan->right_x());
          ++lhs_scan;
        } else {
          interval_offsets.push_back(rhs_scan->left_x());
          interval_offsets.push_back(rhs_scan->right_x());
          ++rhs_scan;
        }
      }

      int k = 0, l = 0, sz_k = interval_offsets.size();
      while (k < sz_k) {
        // Last interval -> insert and break.
        if (k + 2 == sz_k) {
          ScanInterval* scan_inter = merged->add_scan_inter();
          scan_inter->set_y(lhs_y);
          scan_inter->set_left_x(interval_offsets[l]);
          scan_inter->set_right_x(interval_offsets[k + 1]);
          break;
        } else if (interval_offsets[k + 2] - 1 == interval_offsets[k + 1]) {
          // Connected -> skip.
          k += 2;
        } else {
          // Not, connected: Reset and add.
          ScanInterval* scan_inter = merged->add_scan_inter();
          scan_inter->set_y(lhs_y);
          scan_inter->set_left_x(interval_offsets[l]);
          scan_inter->set_right_x(interval_offsets[k + 1]);
          k += 2;
          l = k;
        }
      }
    }
  }
}

void MergeRasterization3D(const Rasterization3D& lhs,
                          const Rasterization3D& rhs,
                          Rasterization3D* merged) {
  Rasterization3D::const_iterator lhs_iter = lhs.begin();
  Rasterization3D::const_iterator rhs_iter = rhs.begin();

  while (lhs_iter != lhs.end() || rhs_iter != rhs.end()) {
    const int lhs_frame = (lhs_iter == lhs.end() ? 1 << 30 : lhs_iter->first);
    const int rhs_frame = (rhs_iter == rhs.end() ? 1 << 30 : rhs_iter->first);

    if (lhs_frame < rhs_frame) {
      shared_ptr<Rasterization> deep_copy(new Rasterization());
      deep_copy->CopyFrom(*lhs_iter->second);
      merged->push_back(std::make_pair(lhs_frame, deep_copy));
      ++lhs_iter;
    } else if (rhs_frame < lhs_frame) {
      shared_ptr<Rasterization> deep_copy(new Rasterization());
      deep_copy->CopyFrom(*rhs_iter->second);
      merged->push_back(std::make_pair(rhs_frame, deep_copy));
      ++rhs_iter;
    } else {
      // Frames are equal.
      //ASSERT_LOG(lhs_frame == rhs_frame);
      shared_ptr<Rasterization> merged_raster(new Rasterization());
      MergeRasterization(*lhs_iter->second,
                         *rhs_iter->second,
                         merged_raster.get());
      merged->push_back(std::make_pair(lhs_frame, merged_raster));
      ++lhs_iter;
      ++rhs_iter;
    }
  }
}

int RasterizationArea(const Rasterization& raster) {
  int area = 0;
  for (RepeatedPtrField<ScanInterval>::const_iterator scan_inter =
           raster.scan_inter().begin();
       scan_inter != raster.scan_inter().end();
       ++scan_inter) {
    area += scan_inter->right_x() - scan_inter->left_x() + 1;
  }
  return area;
}

void DrawShapeDescriptors(const vector<int>& overseg_region_ids,
                          const SegmentationDesc& desc,
                          char* img,
                          int width_step,
                          int width,
                          int height,
                          int channels,
                          const int* region_id) {
  vector<const Region2D*> regions;
  for (vector<int>::const_iterator overseg_region_id = overseg_region_ids.begin();
       overseg_region_id != overseg_region_ids.end();
       ++overseg_region_id) {
    regions.push_back(&GetRegion2DFromId(*overseg_region_id, desc));
  }

  ShapeDescriptor shape_desc;
  if (!GetShapeDescriptorFromRegions(regions, &shape_desc)) {
    return;
  }

  // Draw 10 x 10, clipped line around mixed_x, mixed_y.
  IplImage frame_buffer;
  cvInitImageHeader(&frame_buffer, cvSize(width, height), IPL_DEPTH_8U, channels);
  cvSetData(&frame_buffer, img, width_step);

  if (region_id == 0) {
    CvPoint left = trunc(cvPoint2D32f(shape_desc.center_x,
                                      shape_desc.center_y) -
                         shape_desc.dir_major * channels);
    CvPoint right = trunc(cvPoint2D32f(shape_desc.center_x,
                                       shape_desc.center_y) +
                          shape_desc.dir_major * channels);
    CvPoint top = trunc(cvPoint2D32f(shape_desc.center_x,
                                     shape_desc.center_y) -
                        shape_desc.dir_minor * channels);
    CvPoint bottom = trunc(cvPoint2D32f(shape_desc.center_x,
                                        shape_desc.center_y) +
                           shape_desc.dir_minor * channels);

    cvClipLine(cvSize(width, height), &left, &right);
    cvClipLine(cvSize(width, height), &top, &bottom);

    cvLine(&frame_buffer, left, right, cvScalar(255, 255, 255, 255));
    cvLine(&frame_buffer, top, bottom, cvScalar(255, 255, 255, 255));
  } else {
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8, 0.8);
    char id_string[128];
    std::sprintf(id_string, "%d", *region_id);
    cvPutText(&frame_buffer,
              id_string,
              cvPoint(shape_desc.center_x, shape_desc.center_y),
              &font,
              cvScalar(0));
  }

  // Determine angle.
  const float angle = atan2(shape_desc.dir_major.y, shape_desc.dir_major.x);

  cvEllipse(&frame_buffer,
            cvPoint(shape_desc.center_x, shape_desc.center_y),
            cvSize(shape_desc.mag_major, shape_desc.mag_minor),
            -angle / M_PI * 180.0f,
            0,
            360,
            cvScalar(255, 255, 255, 255));
}

void SetShapeDescriptorFromRegion(Region2D* r) {
  float mean_x = 0, mean_y = 0, moment_xx = 0, moment_yy = 0, moment_xy = 0;
  float area_sum = 0;

  for(RepeatedPtrField<ScanInterval>::const_iterator scan_inter =
          r->raster().scan_inter().begin();
      scan_inter != r->raster().scan_inter().end();
      ++scan_inter) {
    // Add to shape descriptor.
    const float m = scan_inter->left_x();
    const float n = scan_inter->right_x();
    const float curr_y = scan_inter->y();
    const float len = (n - m + 1);
    area_sum += len;
    const float center_x = (n + m) * 0.5;

    mean_x += center_x * len;
    mean_y += curr_y * len;
    moment_xy += curr_y * (center_x * len);
    moment_yy += curr_y * curr_y * len;

    // sum_m^n x_i^2  -->
    // 1/6 * Factor[2 n^3 + 3 n^2 + n  - 2 (m - 1)^3 - 3*(m - 1)^2 - (m - 1)]
    // = 1/6 * (1 + n - m) (-m + 2 m^2 + n + 2 m n + 2 n^2)
    moment_xx += len * (-m + 2 * m * m + n + 2 * m * n + 2 * n * n) / 6.0f;
  }

  const float inv_area = 1.0f / area_sum;
  ShapeMoments* moments = r->mutable_shape_moments();
  moments->set_size(area_sum);
  moments->set_mean_x(mean_x * inv_area);
  moments->set_mean_y(mean_y * inv_area);
  moments->set_moment_xx(moment_xx * inv_area);
  moments->set_moment_xy(moment_xy * inv_area);
  moments->set_moment_yy(moment_yy * inv_area);
}

void ConstrainHierarchyToFrameInterval(int lhs,
                                       int rhs,
                                       const HierarchyLevel& input_hierachy,
                                       HierarchyLevel* constraint_hierarchy) {
  //ASSURE_LOG(constraint_hierarchy);
  unordered_map<int, bool> interval_region;
  // First pass: Determine regions to be removed.
  for (RepeatedPtrField<CompoundRegion>::const_iterator region =
       input_hierachy.region().begin();
       region != input_hierachy.region().end();
       ++region) {
    if (region->start_frame() > rhs || region->end_frame() < lhs) {
      interval_region[region->id()] = true;
    }
  }

  // No regions have to be removed: Simply copy and return.
  if (interval_region.size() == 0) {
    *constraint_hierarchy = input_hierachy;
    return;
  }

  // Copy only regions within interval.
  for (RepeatedPtrField<CompoundRegion>::const_iterator region =
       input_hierachy.region().begin();
       region != input_hierachy.region().end();
       ++region) {
    if (interval_region.find(region->id()) != interval_region.end()) {
      continue;
    }

    CompoundRegion* proto_region = constraint_hierarchy->add_region();
    // Copy.
    *proto_region = *region;

    // Reset neighbors.
    proto_region->clear_neighbor_id();

    for (RepeatedField<int>::const_iterator neighbor =
         region->neighbor_id().begin();
         neighbor != region->neighbor_id().end();
         ++neighbor) {
      if (interval_region.find(*neighbor) == interval_region.end()) {
        proto_region->add_neighbor_id(*neighbor);
      }
    }
  }
}

void SegmentationDescToIdImage(int* img,
                               int width_step,
                               int width,
                               int height,
                               int level,
                               const SegmentationDesc& seg,
                               const Hierarchy* seg_hier) {
  if (level > 0) {
    //ASSURE_LOG(seg_hier) << "Hierarchy level requested but seg_hier not set.";

    if (level > seg_hier->size()) {
      level = seg_hier->size();
      std::cerr << "Clamping requested level to " << level;
    }
  }

  // Fill each region with it's id.
  const RepeatedPtrField<Region2D>& regions = seg.region();
  for (RepeatedPtrField<Region2D>::const_iterator r = regions.begin();
       r != regions.end();
       ++r) {
    // Get id.
    int region_id = r->id();
    if (level != 0) {
      region_id = GetParentId(r->id(), 0, level, *seg_hier);
    }

    for(RepeatedPtrField<ScanInterval>::const_iterator s =
            r->raster().scan_inter().begin();
        s != r->raster().scan_inter().end();
        ++s) {
      int* out_ptr = PtrOffset(img, s->y() * width_step) + s->left_x();
      for (int j = 0, len = s->right_x() - s->left_x() + 1; j < len; ++j, ++out_ptr) {
        *out_ptr = region_id;
      }
    }
  }
}

int GetOversegmentedRegionIdFromPoint(int x, int y, const SegmentationDesc& seg) {
  const RepeatedPtrField<Region2D>& regions = seg.region();
  for(RepeatedPtrField<Region2D>::const_iterator r = regions.begin();
      r != regions.end();
      ++r) {
    RepeatedPtrField<ScanInterval>::const_iterator s = LocateScanLine(y, r->raster());
    while (s != r->raster().scan_inter().end() &&
           s->y() == y) {
      if (x >= s->left_x() && x <= s->right_x()) {
        // Get my id and return.
        return r->id();
      }
      ++s;
    }
  }

  return -1;
}

namespace {

void TruncateHierarchy(int levels, Hierarchy* hierarchy) {
  //ASSURE_LOG(levels > 0) << "At least one level needs to be present.";

  if (hierarchy->size() <= levels) {
    return;
  }

  while (hierarchy->size() > levels) {
    hierarchy->RemoveLast();
  }

  HierarchyLevel* last_level = hierarchy->Mutable(hierarchy->size() - 1);
  for (RepeatedPtrField<CompoundRegion>::iterator region =
           last_level->mutable_region()->begin();
       region != last_level->mutable_region()->end();
       ++region) {
    region->set_parent_id(-1);   // Erase previous parents if necessary.
  }
}

void MergeHierarchyLevel(const HierarchyLevel& level_1,
                         const HierarchyLevel& level_2,
                         HierarchyLevel* merged_level) {
  //ASSURE_LOG(merged_level);
  // Merge compound regions while preserving ordering by id.
  RepeatedPtrField<CompoundRegion>::const_iterator region_1_ptr =
      level_1.region().begin();
  RepeatedPtrField<CompoundRegion>::const_iterator region_2_ptr =
      level_2.region().begin();
  while (region_1_ptr != level_1.region().end() &&
         region_2_ptr != level_2.region().end()) {
    if (region_1_ptr->id() < region_2_ptr->id()) {
      merged_level->mutable_region()->Add()->CopyFrom(*region_1_ptr++);
    } else if (region_2_ptr->id() < region_1_ptr->id()) {
      merged_level->mutable_region()->Add()->CopyFrom(*region_2_ptr++);
    } else {
      // Region id's are equal. Merge compound regions.
      CompoundRegion merged_region;
      merged_region.set_id(region_1_ptr->id());
      merged_region.set_size(region_1_ptr->size() +
                              region_2_ptr->size());
      merged_region.set_parent_id(region_1_ptr->parent_id());
      //ASSURE_LOG(region_1_ptr->parent_id() == region_2_ptr->parent_id());

      // Merge children and neighbors.
      std::set_union(region_1_ptr->neighbor_id().begin(),
                     region_1_ptr->neighbor_id().end(),
                     region_2_ptr->neighbor_id().begin(),
                     region_2_ptr->neighbor_id().end(),
                     google::protobuf::RepeatedFieldBackInserter(
                         merged_region.mutable_neighbor_id()));

      std::set_union(region_1_ptr->child_id().begin(),
                     region_1_ptr->child_id().end(),
                     region_2_ptr->child_id().begin(),
                     region_2_ptr->child_id().end(),
                     google::protobuf::RepeatedFieldBackInserter(
                         merged_region.mutable_child_id()));

      merged_region.set_start_frame(std::min(region_1_ptr->start_frame(),
                                             region_2_ptr->start_frame()));
      merged_region.set_end_frame(std::max(region_1_ptr->end_frame(),
                                           region_2_ptr->end_frame()));

      ++region_1_ptr;
      ++region_2_ptr;
    }
  }

  // Finish remaining regions.
  while (region_1_ptr != level_1.region().end()) {
    merged_level->mutable_region()->Add()->CopyFrom(*region_1_ptr++);
  }

  while (region_2_ptr != level_2.region().end()) {
    merged_level->mutable_region()->Add()->CopyFrom(*region_2_ptr++);
  }
}

} // namespace

void BuildGlobalHierarchy(const Hierarchy& chunk_hierarchy,
                          int chunk_frame_start,
                          Hierarchy* global_hierarchy) {
  // On first call, copy and return.
  if (global_hierarchy->size() == 0) {
    global_hierarchy->CopyFrom(chunk_hierarchy);
    return;
  }

  // Levels need to be compatible.
  if (global_hierarchy->size() > chunk_hierarchy.size()) {
    TruncateHierarchy(chunk_hierarchy.size(), global_hierarchy);
  }

  for (int level = 0; level < global_hierarchy->size(); ++level) {
    const HierarchyLevel& level_1 = global_hierarchy->Get(level);
    HierarchyLevel level_2 = chunk_hierarchy.Get(level);

    bool clear_parent = false;
    // Last level, might have to adjust members in chunk hierarchy.
    if (level + 1 == global_hierarchy->size() &&
        global_hierarchy->size() < chunk_hierarchy.size()) {
      clear_parent = true;
    }

    for (RepeatedPtrField<CompoundRegion>::iterator region =
             level_2.mutable_region()->begin();
         region != level_2.mutable_region()->end();
         ++region) {
      region->set_start_frame(region->start_frame() + chunk_frame_start);
      region->set_end_frame(region->end_frame() + chunk_frame_start);
      if (clear_parent) {
        region->set_parent_id(-1);
      }
    }

    // Merge Levels.
    HierarchyLevel merged_level;
    MergeHierarchyLevel(level_1, level_2, &merged_level);
  }
}

bool VerifyGlobalHierarchy(const Hierarchy& hierarchy) {
  // Check that neighbors as well as parents and children are mutual neighbors.
  //LOG(INFO) << "Verifying global hierarchy.";
  int hier_levels = hierarchy.size();
  for (int level = 0; level < hier_levels; ++level) {
    const HierarchyLevel& curr_level = hierarchy.Get(level);
    for (RepeatedPtrField<CompoundRegion>::const_iterator region =
             curr_level.region().begin();
         region != curr_level.region().end();
         ++region) {
      for (RepeatedField<int>::const_iterator neighbor_id =
               region->neighbor_id().begin();
           neighbor_id != region->neighbor_id().end();
           ++neighbor_id) {
        const CompoundRegion& neighbor = GetCompoundRegionFromId(*neighbor_id,
                                                                 curr_level);
        RepeatedField<int>::const_iterator insert_pos =
            std::lower_bound(neighbor.neighbor_id().begin(),
                             neighbor.neighbor_id().end(),
                             region->id());
        if (insert_pos == neighbor.neighbor_id().end() ||
            *insert_pos != region->id()) {
          //LOG(ERROR) << "Mutual neighbor error for region " << region->id()
          //           << " and neighbor " << *neighbor_id;
          return false;
        }
      }

      if (level + 1 < hier_levels) {
        const HierarchyLevel& next_level = hierarchy.Get(level + 1);
        if (region->parent_id() < 0) {
          //LOG(ERROR) << "Region " << region->id() << " has no parent, but "
          //              " is expected to have one.";
          return false;
        }

        const CompoundRegion& parent = GetCompoundRegionFromId(region->parent_id(),
                                                               next_level);
        RepeatedField<int>::const_iterator insert_pos =
            std::lower_bound(parent.child_id().begin(),
                             parent.child_id().end(),
                             region->id());
        if (insert_pos == parent.child_id().end() ||
            *insert_pos != region->id()) {
          //LOG(ERROR) << "Mutual parent/child error for region " << region->id()
          //           << " and parent " << parent.id();
          return false;
        }
      }

      if (level > 0) {
        const HierarchyLevel& prev_level = hierarchy.Get(level - 1);
        int aggregated_size = 0;
        int aggregated_start = std::numeric_limits<int>::max();
        int aggregated_end = std::numeric_limits<int>::min();

        for (RepeatedField<int>::const_iterator child_id =
               region->child_id().begin();
           child_id != region->child_id().end();
           ++child_id) {
          const CompoundRegion& child = GetCompoundRegionFromId(*child_id,
                                                              prev_level);
           if (child.parent_id() != region->id()) {
             //LOG(ERROR) << "Mutual child parent error for parent region "
             //           << region->id() << " and child " << *child_id;
            return false;
          }

          aggregated_size += child.size();
          aggregated_start = std::min(aggregated_start, child.start_frame());
          aggregated_end = std::max(aggregated_end, child.end_frame());
        }
        if (aggregated_size != region->size()) {
          //LOG(ERROR) << "Child region size does not sum up to region size "
          //           << " for region " << region->id();
          return false;
        }

        if (aggregated_start != region->start_frame() ||
            aggregated_end != region->end_frame()) {
          //LOG(ERROR) << "Aggregated start and end over child regions is "
          //           << "incompatible.";
          return false;
        }
      }
    }  // end regions.
  } // end levels.
  return true;
}

namespace {

bool ScanIntervalsNeighbored(const ScanInterval& lhs,
                             const ScanInterval& rhs) {
  return abs(lhs.y() - rhs.y()) <= 1 &&
        std::max(lhs.left_x(), rhs.left_x()) -
        std::min(lhs.right_x(), rhs.right_x()) <= 1;
}

} // namespace

int ConnectedComponents(const Rasterization& raster,
                        vector<Rasterization>* components) {
  const int scan_inter_size = raster.scan_inter_size();

  // Compute disjoint sets.
  vector<int> ranks(scan_inter_size);
  vector<int> parents(scan_inter_size);
  vector<int> elements;
  boost::disjoint_sets<int*, int*> classes(&ranks[0], &parents[0]);

  int last_change_idx;
  int last_y = -1;
  int test_idx = 0;

  for (int i = 0; i < scan_inter_size; ++i) {
    classes.make_set(i);
    elements[i] = i;

    const ScanInterval& curr_scan = raster.scan_inter(i);
    if (curr_scan.y() != last_y) {
      // If neighboring update test_idx.
      if (last_y + 1 == curr_scan.y()) {
        test_idx = last_change_idx;
      } else {
        test_idx = i;
      }

      last_y = curr_scan.y();
      last_change_idx = i;
    }

    for (int k = test_idx; k < i; ++k) {
      if (ScanIntervalsNeighbored(curr_scan, raster.scan_inter(k))) {
          classes.union_set(i, k);
      }
    }
  }

  int num_components = classes.count_sets(elements.begin(), elements.end());
  if (num_components == 1) {
    return 1;
  }

  if (components != NULL) {
    // Compile component rasterizations.
    unordered_map<int, int> rep_to_component_map;
    for (int i = 0; i < scan_inter_size; ++i) {
       const ScanInterval& curr_scan = raster.scan_inter(i);

       int rep = classes.find_set(i);
       unordered_map<int, int>::const_iterator rep_iter =
           rep_to_component_map.find(rep);
       if (rep_iter == rep_to_component_map.end()) {
         rep_to_component_map[rep] = components->size();
         components->push_back(Rasterization());
         components->back().add_scan_inter()->CopyFrom(curr_scan);
       } else {
         (*components)[rep_iter->second].add_scan_inter()->CopyFrom(curr_scan);
       }
    }
  }

  return num_components;
}

} // namespace videoseg
