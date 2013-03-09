#include "videoseg/tree.hpp"
#include "videoseg/hierarchical-segmentation.hpp"
#include "videoseg/io.hpp"
#include <algorithm>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/visitors.hpp>
#include <glog/logging.h>

namespace videoseg {

////////////////////////////////////////////////////////////////////////////////
// SegmentationNode

SegmentationNode::SegmentationNode() : interior_(), leaf_() {}

void SegmentationNode::swap(SegmentationNode& other) {
  leaf_.swap(other.leaf_);
  interior_.swap(other.interior_);
}

void SegmentationNode::setIsLeaf(bool is_leaf) {
  if (is_leaf) {
    leaf_.reset(new Leaf());
    interior_.reset();
  } else {
    interior_.reset(new Interior());
    leaf_.reset();
  }
}

////////////////////////////////////////////////////////////////////////////////
// loadSegmentation

// For locating a vertex given a CompoundRegion ID.
typedef map<int, VertexIndex> VertexMap;
// Distinct sets of CompoundRegion IDs per hierarchy level.
typedef vector<VertexMap> VertexMapList;

struct CompareCompoundRegion {
  bool operator()(const CompoundRegion& region, int id) const {
    return region.id() < id;
  }

  bool operator()(int id, const CompoundRegion& region) const {
    return id < region.id();
  }
};

// Incorporates a partial hierarchy into the current tree.
void addToHierarchy(SegmentationTree& tree,
                    set<VertexIndex>& roots,
                    VertexMapList& lookups,
                    const Hierarchy& hierarchy) {
  typedef RepeatedPtrField<CompoundRegion> CompoundRegionList;

  // Grow levels to match hierarchy if necessary.
  while (lookups.size() < hierarchy.size()) {
    lookups.push_back(VertexMap());
  }

  // Iterate through leaf nodes.
  const CompoundRegionList& leaves = hierarchy.Get(0).region();
  CompoundRegionList::const_iterator leaf;

  for (leaf = leaves.begin(); leaf != leaves.end(); ++leaf) {

    // Climb hierarchy until we reach the root node.
    const CompoundRegion* hierarchy_node = &*leaf;
    VertexIndex previous_vertex;
    bool reached_root = false;
    int level = 0;
    Hierarchy::const_iterator generation = hierarchy.begin();
    vector<map<int, VertexIndex> >::iterator lookup = lookups.begin();

    while (!reached_root) {
      int id = hierarchy_node->id();
      int parent_id = hierarchy_node->parent_id();

      // Find node, or create one.
      VertexIndex vertex;
      map<int, VertexIndex>::const_iterator it = lookup->find(id);

      if (it == lookup->end()) {
        // Not found. Create a new node.
        vertex = boost::add_vertex(tree);
        if (level == 0) {
          // Create leaf node.
          tree[vertex].setIsLeaf(true);
        } else {
          // Create non-leaf node.
          tree[vertex].setIsLeaf(false);
        }

        // Add it to the lookup.
        (*lookup)[id] = vertex;

        if (level > 0) {
          // Add an edge from the new node down to the previous node.
          boost::add_edge(vertex, previous_vertex, tree);
        }

        if (parent_id < 0) {
          reached_root = true;
          // Add to list of root nodes.
          roots.insert(vertex);
        }
      } else {
        vertex = it->second;
        // Node already exists.

        // If node had no parent and will now, remove from list of roots.
        if (boost::in_degree(vertex, tree) == 0) {
          if (parent_id >= 0) {
            // Remove from the root list and add edge.
            int erased = roots.erase(vertex);
            CHECK_EQ(erased, 1);
          }
        }

        // If the previous node did not have a parent, add the edge.
        if (level > 0) {
          if (boost::in_degree(previous_vertex, tree) == 0) {
            boost::add_edge(vertex, previous_vertex, tree);
          } else {
            // Otherwise, check that it was the same parent!
            SegmentationTree::in_edge_iterator edge;
            boost::tie(edge, boost::tuples::ignore) =
                boost::in_edges(previous_vertex, tree);
            CHECK_EQ(boost::source(*edge, tree), vertex) <<
                "Node had a different parent";
          }
        }
      }

      previous_vertex = vertex;

      // Move up tree.
      if (parent_id < 0) {
        // Next node is the root.
        reached_root = true;
      } else {
        // Move up to next node.
        ++generation;
        ++lookup;
        level += 1;

        CHECK(generation != hierarchy.end()) << "Reached end of hierarchy "
            "levels before root";
        CHECK(lookup != lookups.end()) << "Reached end of node levels before "
            "root";

        // Search through next level for ID.
        const CompoundRegionList& parents = generation->region();
        CompareCompoundRegion less_than;

        // Find actual element.
        CompoundRegionList::const_iterator parent = std::lower_bound(
            parents.begin(), parents.end(), parent_id, less_than);
        CHECK(parent != parents.end() && parent->id() == parent_id) <<
            "Could not locate parent node";
        hierarchy_node = &*parent;
      }
    }
  }
}

// Loads the full hierarchy from a sequence.
void loadHierarchy(SegmentationTree& tree,
                   VertexIndex& root,
                   VertexMapList& lookups,
                   SegmentationReader& reader) {
  // Keep track of top nodes in an ordered list (for finding and removing).
  // Come back and create single root node at the end.
  set<VertexIndex> roots;

  // Read hierarchy.
  for (int t = 0; t < reader.NumFrames(); t += 1) {
    // Load segmentation of next frame from file.
    SegmentationDesc segmentation;
    vector<unsigned char> buffer(reader.ReadNextFrameSize());
    reader.ReadNextFrame(&buffer[0]);
    segmentation.ParseFromArray(&buffer[0], buffer.size());

    // Has hierarchy changed? (or first frame)
    if (t == segmentation.hierarchy_frame_idx()) {
      LOG(INFO) << "Updating hierarchy at frame " << t;
      addToHierarchy(tree, roots, lookups, segmentation.hierarchy());
    }
  }

  if (boost::num_vertices(tree) > 0) {
    // Tree was not empty.
    CHECK(!roots.empty()) << "No root nodes";

    if (roots.size() == 1) {
      // Only one root. Use it.
      root = *roots.begin();
    } else {
      // Merge multiple roots into one final root.
      root = boost::add_vertex(tree);
      tree[root].setIsLeaf(false);

      set<VertexIndex>::const_iterator index;
      for (index = roots.begin(); index != roots.end(); ++index) {
        boost::add_edge(root, *index, tree);
      }
    }
  } else {
    LOG(WARNING) << "Tree is empty";
  }
}

// Fills in a full hierarchy with regions.
void loadRegions(SegmentationTree& tree,
                 VertexIndex root,
                 const VertexMap& leaf_lookup,
                 SegmentationReader& reader) {
  typedef RepeatedPtrField<Region2D> RegionList;

  for (int t = 0; t < reader.NumFrames(); t += 1) {
    // Load segmentation of next frame from file.
    SegmentationDesc segmentation;
    vector<unsigned char> buffer(reader.ReadNextFrameSize());
    reader.ReadNextFrame(&buffer[0]);
    segmentation.ParseFromArray(&buffer[0], buffer.size());

    // Iterate through regions in this frame.
    const RegionList& regions = segmentation.region();
    RegionList::const_iterator region;

    for (region = regions.begin(); region != regions.end(); ++region) {
      // Find element in first level.
      int id = region->id();

      // Find node.
      VertexMap::const_iterator it = leaf_lookup.find(id);
      CHECK(it != leaf_lookup.end()) << "Could not find leaf node in tree";
      VertexIndex index = it->second;
      SegmentationNode& node = tree[index];

      // Copy region into tree.
      VideoRegion::FrameList& frames = node.leaf().region.frames();
      frames[t] = region->raster();

      // Now step up hierarchy to ensure that frame sets are correct.
      bool reached_root = false;

      while (!reached_root) {
        if (boost::in_degree(index, tree) == 0) {
          reached_root = true;
        } else {
          // Get parent.
          SegmentationTree::in_edge_iterator edge;
          boost::tie(edge, boost::tuples::ignore) = boost::in_edges(index,
              tree);
          index = boost::source(*edge, tree);

          // Ensure that frame is in set.
          tree[index].interior().frames.insert(t);
        }
      }
    }
  }
}

bool loadSegmentation(const string& filename,
                      SegmentationTree& tree,
                      VertexIndex& root,
                      int& num_frames,
                      int& width,
                      int& height) {
  // Read segmentation file.
  SegmentationReader reader(filename);
  bool ok = reader.OpenFileAndReadHeaders();
  if (!ok) {
    LOG(WARNING) << "Could not read headers from \"" << filename << "\"";
    return false;
  }

  num_frames = reader.NumFrames();
  LOG(INFO) << "Contains " << num_frames << " frames";

  // Load segmentation of next frame from file.
  SegmentationDesc segmentation;
  vector<unsigned char> buffer(reader.ReadNextFrameSize());
  reader.ReadNextFrame(&buffer[0]);
  segmentation.ParseFromArray(&buffer[0], buffer.size());

  width = segmentation.frame_width();
  height = segmentation.frame_height();

  // A vertex lookup for each level of the hierarchy.
  VertexMapList lookups;

  LOG(INFO) << "Loading hierarchy...";
  reader.SeekToFrame(0);
  loadHierarchy(tree, root, lookups, reader);

  LOG(INFO) << "Loading regions...";
  reader.SeekToFrame(0);
  loadRegions(tree, root, lookups.front(), reader);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// visitRegions

// Terminates traversal when no descendants are present in frame t.
struct NotPresentInFrameTerminator {
  int t;

  bool operator()(VertexIndex index, const SegmentationTree& graph) const {
    bool present;
    const SegmentationNode& node = graph[index];

    if (node.isLeaf()) {
      // Check if node present in this frame.
      const VideoRegion::FrameList& frames = node.leaf().region.frames();
      present = (frames.find(t) != frames.end());
    } else {
      // Check if interior node contains this frame.
      const set<int>& frames = node.interior().frames;
      present = (frames.find(t) != frames.end());
    }

    return !present;
  }

  NotPresentInFrameTerminator(int t) : t(t) {}
};

// Visits a leaf if it is present in frame t.
struct FrameVisitor : public boost::base_visitor<FrameVisitor> {
  int t;
  VisitRegion* function;

  // Call when first visiting vertex.
  typedef boost::on_discover_vertex event_filter;

  void operator()(VertexIndex index, const SegmentationTree& tree) {
    const SegmentationNode& node = tree[index];

    if (node.isLeaf()) {
      // Check if node present in this frame.
      const VideoRegion::FrameList& frames = node.leaf().region.frames();
      VideoRegion::FrameList::const_iterator frame = frames.find(t);
      bool present = (frame != frames.end());

      if (present) {
        (*function)(index, frame->second);
      }
    }
  }

  FrameVisitor(int t, VisitRegion& function)
      : t(t), function(&function) {}
};

void visitRegions(VertexIndex index,
                  const SegmentationTree& tree,
                  int t,
                  VisitRegion& function) {
  // Function to draw region.
  FrameVisitor visitor(t, function);
  // Terminate when no nodes are present in this frame.
  NotPresentInFrameTerminator terminator(t);

  // To avoid re-visiting (not required for tree).
  typedef map<VertexIndex, boost::default_color_type> ColorMap;
  ColorMap color_map;
  boost::associative_property_map<ColorMap> color_property_map(color_map);

  // Traverse tree below node.
  boost::depth_first_visit(tree, index, boost::make_dfs_visitor(visitor),
      color_property_map, terminator);
}

}
