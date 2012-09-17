#include <deque>
#include <vector>
#include <algorithm>
#include <stack>
#include <set>
#include <string>
#include <cstdlib>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "sift_feature.hpp"
#include "multiview_track_list.hpp"
#include "kmeans.hpp"
#include "util.hpp"

#include "read_lines.hpp"
#include "sift_feature_reader.hpp"
#include "iterator_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "default_writer.hpp"

const int MIN_TRACK_SIZE = 2;

DEFINE_int32(k, 2, "Branching factor");

class Feature : public KMeansPoint {
  public:
    Frame frame;
    int id;
    Descriptor descriptor;

    Feature() : frame(), id(-1), descriptor() {}

    Feature(const Frame& frame, int id)
        : frame(frame), id(id), descriptor() {}

    Feature(const Frame& frame, int id, const Descriptor& descriptor)
        : frame(frame), id(id), descriptor(descriptor) {}

    void swap(Feature& other) {
      std::swap(frame, other.frame);
      std::swap(id, other.id);
      descriptor.swap(other.descriptor);
    }

    const std::vector<double>& vector() const {
      return descriptor.data;
    }
};

typedef std::vector<const Feature*> FeatureSubset;

std::string makeFilename(const std::string& format,
                         const std::string& view,
                         int time) {
  return boost::str(boost::format(format) % view % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Finds tracks by clustering in feature space." << std::endl;
  usage << std::endl;
  usage << argv[0] << " descriptors-format view-names num-frames tracks" <<
      std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 5) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

void addImageFeaturesToList(std::deque<SiftFeature>& features,
                            const Frame& frame,
                            std::deque<Feature>& global) {
  int i = 0;
  std::deque<SiftFeature>::iterator feature;
  for (feature = features.begin(); feature != features.end(); ++feature) {
    // Construct global feature.
    Feature global_feature(frame, i);
    // Swap in descriptor to save copying.
    global_feature.descriptor.swap(feature->descriptor);

    // Add to list by swapping.
    global.push_back(Feature());
    global.back().swap(global_feature);

    i += 1;
  }
}

bool loadFeatures(const std::string& format,
                  const std::vector<std::string>& views,
                  int num_frames,
                  std::deque<Feature>& features) {
  features.clear();

  int v = 0;
  std::vector<std::string>::const_iterator view;
  for (view = views.begin(); view != views.end(); ++view) {
    for (int t = 0; t < num_frames; t += 1) {
      // Load features for this image.
      std::deque<SiftFeature> image_features;
      SiftFeatureReader reader;
      std::string file = makeFilename(format, *view, t);
      bool ok = loadList(file, image_features, reader);
      if (!ok) {
        return false;
      }
      DLOG(INFO) << "Loaded " << image_features.size() << " features for (" <<
          v << ", " << t << ")";

      // Copy features into big list.
      Frame frame(v, t);
      addImageFeaturesToList(image_features, frame, features);
    }

    v += 1;
  }

  return true;
}

bool isConsistent(const FeatureSubset& features) {
  std::set<Frame> visible;

  // Iterate through all points.
  FeatureSubset::const_iterator feature;
  for (feature = features.begin(); feature != features.end(); ++feature) {
    const Frame& frame = (*feature)->frame;

    // Check if the feature has already been observed in this frame.
    if (visible.find(frame) != visible.end()) {
      // It has! Inconsistent!
      return false;
    }

    // If not then it has now.
    visible.insert(frame);
  }

  return true;
}

std::string splitReport(const FeatureSubset& features,
                        int k,
                        const std::deque<FeatureSubset>& children) {
  std::ostringstream ss;
  ss << "Divided " << features.size() << " features into (";

  std::deque<FeatureSubset>::const_iterator child;
  for (child = children.begin(); child != children.end(); ++child) {
    if (child != children.begin()) {
      ss << ", ";
    }
    ss << child->size();
  }
  ss << ")";

  return ss.str();
}

void split(const FeatureSubset& features,
           int k,
           std::deque<FeatureSubset>& children,
           boost::random::mt19937& generator) {
  // Convert to k-means-compatible interface.
  std::vector<const KMeansPoint*> points;
  std::transform(features.begin(), features.end(), std::back_inserter(points),
      cast<const KMeansPoint*, const Feature*>);

  // Cluster descriptors with random initialization.
  std::deque<std::vector<double> > centers;
  std::vector<int> labels;
  randomKMeans(points, k, centers, labels, generator);

  // Sort into children using labels.
  children.assign(k, FeatureSubset());

  CHECK(labels.size() == features.size());
  FeatureSubset::const_iterator feature = features.begin();
  std::vector<int>::const_iterator label = labels.begin();

  while (feature != features.end()) {
    children[*label].push_back(*feature);

    ++feature;
    ++label;
  }

  // Print report.
  LOG(INFO) << splitReport(features, k, children);
}

void featuresToTrack(const FeatureSubset& features,
                     MultiviewTrack<int>& track,
                     int num_views) {
  track = MultiviewTrack<int>(num_views);

  FeatureSubset::const_iterator feature;
  for (feature = features.begin(); feature != features.end(); ++feature) {
    const Frame& frame = (*feature)->frame;
    track.view(frame.view)[frame.time] = (*feature)->id;
  }
}

void subsetsToTracks(const std::deque<FeatureSubset>& subsets,
                     MultiviewTrackList<int>& tracks,
                     int num_views) {
  tracks = MultiviewTrackList<int>(num_views);

  std::deque<FeatureSubset>::const_iterator subset;
  for (subset = subsets.begin(); subset != subsets.end(); ++subset) {
    MultiviewTrack<int> track;
    featuresToTrack(*subset, track, num_views);

    tracks.push_back(MultiviewTrack<int>());
    tracks.back().swap(track);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string descriptors_format = argv[1];
  std::string views_file = argv[2];
  int num_frames = boost::lexical_cast<int>(argv[3]);
  std::string tracks_file = argv[4];

  bool ok;

  // Load names of views.
  std::vector<std::string> views;
  ok = readLines(views_file, views);
  CHECK(ok) << "Could not load view names";

  // Load descriptors for every feature.
  std::deque<Feature> features;
  ok = loadFeatures(descriptors_format, views, num_frames, features);
  CHECK(ok) << "Could not load features";

  boost::random::mt19937 generator;

  // Divide the features into validsets.
  std::deque<FeatureSubset> valid;

  // Maintin a queue of subsets which need to be clustered.
  std::stack<FeatureSubset> pending;
  // Initially put all features in one set.
  pending.push(FeatureSubset());
  std::transform(features.begin(), features.end(),
      std::back_inserter(pending.top()), takeAddress<const Feature>);

  while (!pending.empty()) {
    // Remove next element from the waiting list.
    FeatureSubset subset;
    subset.swap(pending.top());
    pending.pop();

    int num_features = subset.size();

    // Check that cluster is large enough to constitute a track.
    if (num_features < MIN_TRACK_SIZE) {
      DLOG(INFO) << "Cluster too small (" << num_features << " < " <<
          MIN_TRACK_SIZE << ")";
      continue;
    }

    if (isConsistent(subset)) {
      // Move to valid list.
      valid.push_back(FeatureSubset());
      valid.back().swap(subset);
      DLOG(INFO) << "Found consistent cluster (" << num_features <<
          " features)";
      continue;
    }

    // Check that cluster is large enough to split.
    if (num_features < FLAGS_k) {
      DLOG(INFO) << "Less points than branching factor";
      continue;
    }

    // Inconsistent. Run k-means.
    std::deque<FeatureSubset> children;
    split(subset, FLAGS_k, children, generator);

    // Add every child to the stack (by swapping).
    std::deque<FeatureSubset>::iterator child;
    for (child = children.begin(); child != children.end(); ++child) {
      pending.push(FeatureSubset());
      pending.top().swap(*child);
    }
  }

  LOG(INFO) << "Found " << valid.size() << " valid clusters";

  // Write subsets out as index tracks.
  MultiviewTrackList<int> tracks;
  int num_views = views.size();
  subsetsToTracks(valid, tracks, num_views);

  DefaultWriter<int> writer;
  ok = saveMultiviewTrackList(tracks_file, tracks, writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
