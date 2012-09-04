#include "vector_reader.hpp"
#include "multiview_track_reader.hpp"
#include "track_reader.hpp"

template<class T>
MultiviewTrackListReader<T>::MultiviewTrackListReader(Reader<T>& reader)
    : reader_(&reader) {}

template<class T>
MultiviewTrackListReader<T>::~MultiviewTrackListReader() {}

template<class T>
bool MultiviewTrackListReader<T>::read(const cv::FileNode& node,
                                       MultiviewTrackList<T>& tracks) {
  // Check node is not empty.
  if (node.type() == cv::FileNode::NONE) {
    LOG(WARNING) << "Empty file node";
    return false;
  }

  // Check that node is a map.
  if (node.type() != cv::FileNode::MAP) {
    LOG(WARNING) << "Expected file node to be a map";
    return false;
  }

  // Get number of views.
  int num_views;
  if (!::read<int>(node["num_views"], num_views)) {
    LOG(WARNING) << "Could not read number of views";
    return false;
  }

  // Initialize data structure.
  tracks = MultiviewTrackList<T>(num_views);

  // Read tracks into vector.
  MultiviewTrackReader<T> track_reader(*reader_, num_views);
  if (!readSequence(node["tracks"], track_reader, std::back_inserter(tracks))) {
    return false;
  }

  return true;
}

template<class T>
bool loadMultiviewTrackList(const std::string& filename,
                            MultiviewTrackList<T>& tracks,
                            Reader<T>& reader) {
  MultiviewTrackListReader<T> list_reader(reader);
  return load(filename, tracks, list_reader);
}
