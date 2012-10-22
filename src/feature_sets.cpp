#include "feature_sets.hpp"
#include <glog/logging.h>

FeatureSets::FeatureSets(const MatchGraph& graph) : features_(), sets_() {
  //SetList::iterator set;

  MatchGraph::vertex_iterator vertex;
  MatchGraph::vertex_iterator end;
  boost::tie(vertex, end) = boost::vertices(graph);
  int i = 0;

  for (; vertex != end; ++vertex) {
    // Make sure that vertices are indexed 0 to n - 1.
    CHECK(int(*vertex) == i);

    // Add a new set containing element i.
    const FeatureIndex& feature = graph[*vertex];
    sets_[i][Frame(feature.view, feature.time)] = i;
    CHECK(sets_[i].size() == 1);

    /*
    if (i == 0) {
      // Initialize the iterator.
      set = sets_.begin();
    } else {
      // Advance the iterator.
      ++set;
    }
    */

    // Add a pointer back to the set.
    features_.push_back(i);

    i += 1;
  }
}

int FeatureSets::count() const {
  return sets_.size();
}

void FeatureSets::join(int u, int v) {
  if (together(u, v)) {
    return;
  }

  SetList::iterator s = sets_.find(features_[u]);
  SetList::iterator t = sets_.find(features_[v]);
  CHECK(s != sets_.end());
  CHECK(t != sets_.end());

  // Make sure we're merging the smaller set into the larger one.
  if (s->second.size() < t->second.size()) {
    std::swap(u, v);
    std::swap(s, t);
  }

  // Merge t into s.
  s->second.insert(t->second.begin(), t->second.end());

  // Set the elements of t to point at s.
  Set::const_iterator e;
  for (e = t->second.begin(); e != t->second.end(); ++e) {
    features_[e->second] = s->first;
  }

  // Remove set t.
  sets_.erase(t);
}

const FeatureSets::Set& FeatureSets::find(int v) const {
  SetList::const_iterator set = sets_.find(features_[v]);
  CHECK(set != sets_.end());
  return set->second;
}

bool FeatureSets::together(int u, int v) const {
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

bool compare(const FeatureSets::Set::value_type& lhs,
             const FeatureSets::Set::value_type& rhs) {
  return lhs.first < rhs.first;
}

}

bool FeatureSets::compatible(int u, int v) const {
  // Assume that u and v are in different sets.
  CHECK(!together(u, v));

  const Set& s = find(u);
  const Set& t = find(v);

  return !overlap(s.begin(), s.end(), t.begin(), t.end(), compare);
}

FeatureSets::const_iterator FeatureSets::begin() const {
  return sets_.begin();
}

FeatureSets::const_iterator FeatureSets::end() const {
  return sets_.end();
}
