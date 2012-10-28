#include "feature_sets.hpp"
#include <glog/logging.h>

template<class T>
FeatureSets<T>::FeatureSets() : features_(), sets_() {}

template<class T>
void FeatureSets<T>::init(const std::vector<ImageIndex>& vertices) {
  std::vector<ImageIndex>::const_iterator vertex;
  int i = 0;

  for (vertex = vertices.begin(); vertex != vertices.end(); ++vertex) {
    // Add a new set containing element i.
    Set& set = sets_[i];
    set.elements[*vertex] = i;

    // Add a pointer back to the set.
    features_.push_back(i);

    i += 1;
  }
}

template<class T>
void FeatureSets<T>::init(const std::vector<ImageIndex>& vertices,
                          const MultiviewTrackList<int>& tracks,
                          const std::map<FeatureIndex, int>& lookup) {
  init(vertices);

  MultiviewTrackList<int>::const_iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Merge first vertex with every other.
    bool first = true;
    int first_vertex = 0;

    MultiviewTrack<int>::FeatureIterator iter(*track);
    for (iter.begin(); !iter.end(); iter.next()) {
      ImageIndex frame = iter.get().first;
      int id = *iter.get().second;
      FeatureIndex feature(frame.view, frame.time, id);

      // Find vertex index in reverse lookup.
      std::map<FeatureIndex, int>::const_iterator entry = lookup.find(feature);
      // Make sure that the entry exists.
      CHECK(entry != lookup.end());
      int vertex = entry->second;

      if (first) {
        first_vertex = vertex;
      } else {
        // Join the two components.
        join(first_vertex, vertex);
      }

      first = false;
    }
  }
}

template<class T>
int FeatureSets<T>::count() const {
  return sets_.size();
}

template<class T>
void FeatureSets<T>::join(int u, int v) {
  if (together(u, v)) {
    return;
  }

  typename SetList::iterator s_iter = sets_.find(features_[u]);
  typename SetList::iterator t_iter = sets_.find(features_[v]);
  CHECK(s_iter != sets_.end());
  CHECK(t_iter != sets_.end());

  // Make sure we're merging the smaller set into the larger one.
  if (s_iter->second.elements.size() < t_iter->second.elements.size()) {
    std::swap(u, v);
    std::swap(s_iter, t_iter);
  }

  Set& s = s_iter->second;;
  Set& t = t_iter->second;;

  // Merge t into s.
  s.elements.insert(t.elements.begin(), t.elements.end());

  // Set the elements of t to point at s.
  std::map<ImageIndex, int>::const_iterator e;
  for (e = t.elements.begin(); e != t.elements.end(); ++e) {
    features_[e->second] = s_iter->first;
  }

  // Remove set t.
  sets_.erase(t_iter);
}

template<class T>
typename FeatureSets<T>::Set& FeatureSets<T>::get(int v) {
  typename SetList::iterator set = sets_.find(features_[v]);
  CHECK(set != sets_.end());
  return set->second;
}

template<class T>
const typename FeatureSets<T>::Set& FeatureSets<T>::find(int v) const {
  typename SetList::const_iterator set = sets_.find(features_[v]);
  CHECK(set != sets_.end());
  return set->second;
}

template<class T>
bool FeatureSets<T>::together(int u, int v) const {
  return features_[u] == features_[v];
}

namespace {

// Adapted from http://www.cplusplus.com/reference/algorithm/set_intersection/
template <class InputIterator1, class InputIterator2, class Compare>
bool overlap(InputIterator1 iter1,
             InputIterator1 last1,
             InputIterator2 iter2,
             InputIterator2 last2,
             Compare comp) {
  while (iter1 != last1 && iter2 != last2) {
    if (comp(*iter1, *iter2)) {
      ++iter1;
    } else if (comp(*iter2, *iter1)) {
      ++iter2;
    } else {
      // Keys are equal.
      return true;
    }
  }

  return false;
}

bool compare(const std::map<ImageIndex, int>::value_type& lhs,
             const std::map<ImageIndex, int>::value_type& rhs) {
  return lhs.first < rhs.first;
}

}

template<class T>
bool FeatureSets<T>::compatible(int u, int v) const {
  // Assume that u and v are in different sets.
  CHECK(!together(u, v));

  const Set& s = find(u);
  const Set& t = find(v);

  return !overlap(s.elements.begin(), s.elements.end(), t.elements.begin(),
      t.elements.end(), compare);
}

template<class T>
T& FeatureSets<T>::property(int v) {
  return get(v).property;
}

template<class T>
const T& FeatureSets<T>::property(int v) const {
  return find(v).property;
}

template<class T>
typename FeatureSets<T>::const_iterator
FeatureSets<T>::begin() const {
  return sets_.begin();
}

template<class T>
typename FeatureSets<T>::const_iterator
FeatureSets<T>::end() const {
  return sets_.end();
}
