#ifndef VIDEOSEG_TREE_HPP_
#define VIDEOSEG_TREE_HPP_

#include "videoseg/using.hpp"
#include "videoseg/region.hpp"
#include "videoseg/video-region.hpp"
#include <boost/graph/adjacency_list.hpp>

namespace videoseg {

// One node of the segmentation tree.
class SegmentationNode {
  public:
    struct Interior {
      set<int> frames;
    };

    struct Leaf {
      VideoRegion region;
    };

    SegmentationNode();

    void swap(SegmentationNode& other);

    inline bool isLeaf() const;
    void setIsLeaf(bool is_leaf);

    inline const Leaf& leaf() const;
    inline Leaf& leaf();
    inline const Interior& interior() const;
    inline Interior& interior();

  private:
    // Non-leaf nodes contain a set of the frames spanned by nodes below it.
    scoped_ptr<Interior> interior_;
    // Leaf nodes specify a spacetime pixel region.
    scoped_ptr<Leaf> leaf_;
};

// Segmentation graph. Leaf nodes contain 2D region.
typedef boost::adjacency_list<boost::setS,
                              boost::setS,
                              boost::bidirectionalS,
                              SegmentationNode>
    SegmentationTree;

typedef SegmentationTree::vertex_descriptor VertexIndex;

// Load segmentation from file.
bool loadSegmentation(const string& filename,
                      SegmentationTree& tree,
                      VertexIndex& root,
                      int& num_frames,
                      int& width,
                      int& height);

// Functor interface for performing an operation on a region.
class VisitRegion {
  public:
    virtual ~VisitRegion() {}
    virtual void operator()(VertexIndex vertex,
                            const Rasterization& region) const = 0;
};

// Method for visiting all regions which belong to descendants of a node and
// appear in frame t.
void visitRegions(VertexIndex index,
                  const SegmentationTree& tree,
                  int t,
                  VisitRegion& function);

} // namespace videoseg

#include "videoseg/tree.inl"

#endif
