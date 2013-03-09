#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "videoseg/region.hpp"
#include "videoseg/draw-region.hpp"
#include "videoseg/tree.hpp"
#include "videoseg/segmentation.hpp"
#include "util/random-color.hpp"
#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace videoseg;

const int FOREGROUND_LABEL = 0;
const int BACKGROUND_LABEL = 1;

const char WINDOW_NAME[] = "Segmentation";
const double SATURATION = 0.9;
const double BRIGHTNESS = 0.8;

typedef map<VertexIndex, cv::Vec3b> ColorMap;
typedef cv::Mat_<VertexIndex> PixelOwnerMap;

void setPixelOwners(const Rasterization& region,
                    PixelOwnerMap& owners,
                    VertexIndex vertex) {
  typedef RepeatedPtrField<ScanInterval> IntervalList;
  const IntervalList& intervals = region.scan_inter();

  IntervalList::const_iterator interval;
  for (interval = intervals.begin(); interval != intervals.end(); ++interval) {
    int i = interval->y();
    for (int j = interval->left_x(); j <= interval->right_x(); j += 1) {
      owners(i, j) = vertex;
    }
  }
}

struct DrawRegion : public VisitRegion {
  cv::Mat* dst;
  bool foreground;
  const cv::Mat* src;
  cv::Vec3b color;
  bool active;
  bool preview;
  PixelOwnerMap* owner_map;
  VertexIndex leaf;

  // Function to draw region.
  void operator()(VertexIndex vertex, const Rasterization& region) const {
    if (foreground) {
      copyRegion(region, *src, *dst);

      if (!preview) {
        if (active) {
          drawRegionBoundary(region, *dst, cv::Vec3b(0, 0, 0));
        }
      }
    } else {
      if (preview) {
        fillRegion(region, *dst, cv::Vec3b(0, 0, 0));
      } else {
        fillRegion(region, *dst, color);
        if (active) {
          cv::Vec3b the_dark_side = color / 2;
          drawRegionBoundary(region, *dst, the_dark_side);
        }
      }
    }

    setPixelOwners(region, *owner_map, leaf);
  }

  DrawRegion(cv::Mat& dst,
             bool foreground,
             const cv::Mat& src,
             cv::Vec3b color,
             bool active,
             bool preview,
             PixelOwnerMap& owner_map,
             VertexIndex leaf)
      : dst(&dst),
        foreground(foreground),
        src(&src),
        color(color),
        active(active),
        preview(preview),
        owner_map(&owner_map),
        leaf(leaf) {}
};

void drawRegion(VertexIndex index,
                const SegmentationTree& tree,
                int t,
                cv::Mat& dst,
                bool foreground,
                const cv::Mat& src,
                cv::Vec3b color,
                bool active,
                bool preview,
                PixelOwnerMap& owner_map) {
  // Function to draw region.
  DrawRegion draw(dst, foreground, src, color, active, preview, owner_map,
      index);
  visitRegions(index, tree, t, draw);
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Interactively segments the foreground in a video." << std::endl;
  usage << std::endl;
  usage << argv[0] << " over-segmentation video foreground-segmentation" <<
      std::endl;
  usage << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

void renderFrame(const map<VertexIndex, int>& leaves,
                 bool have_active_leaf,
                 VertexIndex active_leaf,
                 bool preview,
                 const SegmentationTree& tree,
                 int t,
                 cv::Mat& display,
                 const ColorMap& colors,
                 const cv::Mat& image,
                 PixelOwnerMap& owners) {
  // Iterate through leaves.
  map<VertexIndex, int>::const_iterator leaf;
  for (leaf = leaves.begin(); leaf != leaves.end(); ++leaf) {
    VertexIndex index = leaf->first;
    bool foreground = (leaf->second == FOREGROUND_LABEL);
    bool selected = (have_active_leaf && leaf->first == active_leaf);

    // Find color.
    ColorMap::const_iterator it = colors.find(leaf->first);
    CHECK(it != colors.end()) << "No color found for vertex";
    cv::Vec3b color = it->second;

    drawRegion(index, tree, t, display, foreground, image, color, selected,
        preview, owners);
  }
}

struct State {
  const SegmentationTree* tree;
  map<VertexIndex, int>* leaves;
  PixelOwnerMap* owners;
  const int* t;
  bool* have_active_leaf;
  VertexIndex* active_leaf;
  bool* preview;
};

void onMouse(int event, int x, int y, int, void* tag) {
  State* state = static_cast<State*>(tag);

  const SegmentationTree& tree = *state->tree;
  map<VertexIndex, int>& leaves = *state->leaves;
  PixelOwnerMap& owners = *state->owners;

  // Find which leaf is under the mouse.
  VertexIndex selected = owners(y, x);
  *state->active_leaf = selected;
  *state->have_active_leaf = true;

  if (event == cv::EVENT_LBUTTONDOWN) {
    if (*state->preview) {
      *state->preview = false;
    } else {
      // Find corresponding entry in leaf set.
      map<VertexIndex, int>::iterator leaf = leaves.find(selected);
      CHECK(leaf != leaves.end()) << "Pixel owner not contained in leaf set";

      // Change label.
      int label = leaf->second;
      label = (label + 1) % 2;
      leaf->second = label;
    }
  } else if (event == cv::EVENT_RBUTTONDOWN) {
    if (*state->preview) {
      *state->preview = false;
    } else {
      // Split node into its children (if it has any).
      if (boost::out_degree(selected, tree) > 0) {
        // Add children to list.
        SegmentationTree::out_edge_iterator edge;
        SegmentationTree::out_edge_iterator end;
        boost::tie(edge, end) = boost::out_edges(selected, tree);

        // Find corresponding entry in leaf set.
        map<VertexIndex, int>::iterator leaf = leaves.find(selected);
        CHECK(leaf != leaves.end()) << "Pixel owner not contained in leaf set";
        // Extract label.
        int label = leaf->second;
        // Remove selected node from list.
        leaves.erase(leaf);
        // Add children to list.
        for (; edge != end; ++edge) {
          VertexIndex child = boost::target(*edge, tree);
          leaves[child] = label;
        }
      }
    }
  }
}

void invertLabel(map<VertexIndex, int>::reference& leaf) {
  leaf.second = 1 - leaf.second;
}

void invertLabels(map<VertexIndex, int>& leaves) {
  std::for_each(leaves.begin(), leaves.end(), invertLabel);
}

int main(int argc, char** argv) {
  init(argc, argv);

  string over_seg_file = argv[1];
  string video_file = argv[2];
  string foreground_seg_file = argv[3];

  bool ok;

  // Open video stream.
  cv::VideoCapture capture;
  ok = capture.open(video_file);
  CHECK(ok) << "Could not open video stream";

  // Extract segmentation tree.
  int num_frames;
  int width;
  int height;
  SegmentationTree tree;
  VertexIndex root;
  ok = loadSegmentation(over_seg_file, tree, root, num_frames, width, height);
  CHECK(ok) << "Could not load segmentation";

  // Current segmentation stored as list of leaves.
  map<VertexIndex, int> leaves;
  // Initialize to children of root (one region uninteresting).
  {
    SegmentationTree::out_edge_iterator edge;
    SegmentationTree::out_edge_iterator end;
    boost::tie(edge, end) = boost::out_edges(root, tree);
    for (; edge != end; ++edge) {
      leaves[boost::target(*edge, tree)] = BACKGROUND_LABEL;
    }
  }

  // Generate a random color for each vertex.
  ColorMap colors;
  {
    SegmentationTree::vertex_iterator vertex;
    SegmentationTree::vertex_iterator end;
    boost::tie(vertex, end) = boost::vertices(tree);
    for (; vertex != end; ++vertex) {
      colors[*vertex] = randomColor(SATURATION, BRIGHTNESS);
    }
  }

  cv::Mat image;

  // Display first frame.
  cv::Mat display = cv::Mat_<cv::Vec3b>(height, width);

  bool exit = false;
  bool pause = false;
  bool preview = false;
  int t = 0;

  bool have_active_leaf;
  VertexIndex active_leaf;
  PixelOwnerMap owners(height, width);

  State state;
  state.tree = &tree;
  state.leaves = &leaves;
  state.owners = &owners;
  state.t = &t;
  state.have_active_leaf = &have_active_leaf;
  state.active_leaf = &active_leaf;
  state.preview = &preview;

  cv::namedWindow(WINDOW_NAME);
  cv::setMouseCallback(WINDOW_NAME, onMouse, &state);

  bool need_to_read_image = true;

  while (!exit) {
    if (need_to_read_image) {
      // Not paused. Counter must have been advanced. Read next image.
      ok = capture.read(image);
      CHECK(ok) << "Could not read frame";
      need_to_read_image = false;
    }

    // Show image.
    renderFrame(leaves, have_active_leaf, active_leaf, preview, tree, t,
        display, colors, image, owners);
    cv::imshow(WINDOW_NAME, display);

    char c = cv::waitKey(30);

    if (c >= 0) {
      if (c == 'q') {
        exit = true;
      } else if (c == ' ') {
        pause = !pause;
      } else if (c == 0x09) {
        // Tab key.
        // Invert all labels.
        invertLabels(leaves);
      } else if (c == 0x0d) {
        // Enter key.
        preview = !preview;
      } else if (c == 0x1b) {
        // Escape. Deselect.
        have_active_leaf = false;
      } else if (c == '0') {
        // Pause and go to first frame.
        pause = true;
        t = 0;
        need_to_read_image = true;
        // Set counter manually.
        capture.set(CV_CAP_PROP_POS_FRAMES, t);
      } else if (c == '$') {
        // Pause and go to last frame.
        pause = true;
        t = num_frames - 1;
        need_to_read_image = true;
        // Set counter manually.
        capture.set(CV_CAP_PROP_POS_FRAMES, t);
      } else if (c == 'j') {
        pause = true;
        // Advance to next frame.
        t = (t + 1) % num_frames;
        need_to_read_image = true;
        // Set counter manually if necessary.
        if (t == 0) {
          capture.set(CV_CAP_PROP_POS_FRAMES, t);
        }
      } else if (c == 'k') {
        pause = true;
        // Go back to previous frame.
        t = (t - 1 + num_frames) % num_frames;
        need_to_read_image = true;
        // Set counter manually.
        capture.set(CV_CAP_PROP_POS_FRAMES, t);
      }
    }

    if (!pause) {
      // Advance frame counter.
      t = (t + 1) % num_frames;
      need_to_read_image = true;
      // Reset video stream if necessary.
      if (t == 0) {
        capture.set(CV_CAP_PROP_POS_FRAMES, t);
      }
    }
  }

  LOG(INFO) << "Flattening segmentation...";
  VideoSegmentation foreground_seg;
  flattenHierarchicalSegmentation(tree, leaves, 2, num_frames, foreground_seg);

  LOG(INFO) << "Saving segmentation...";
  std::ofstream ofs(foreground_seg_file.c_str(),
      std::ios::out | std::ios::trunc | std::ios::binary);
  ok = foreground_seg.SerializeToOstream(&ofs);
  CHECK(ok) << "Could not save output";

  return 0;
}
