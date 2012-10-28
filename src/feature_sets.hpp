#ifndef FEATURE_SETS_HPP_
#define FEATURE_SETS_HPP_

#include <map>
#include <vector>
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"
#include "feature_index.hpp"

// Templated because it is essentially a container.
// The property of a set is accessed by any member in the set.
// TODO: Provide constant time find().

template<class T>
class FeatureSets {
  public:
    struct Set {
      std::map<ImageIndex, int> elements;
      T property;
    };

    FeatureSets();
    // Add each vertex to its own feature set.
    void init(const std::vector<ImageIndex>& vertices);
    // Create a vertex set for each track.
    // If a vertex does not appear in a track, it gets its own set.
    //
    // Parameters:
    // vertices -- The frame that each vertex was observed in.
    // tracks -- Multiview tracks, containing the index of the feature within
    //   its image.
    // lookup -- A lookup from feature (view, time, id) to vertex index.
    void init(const std::vector<ImageIndex>& vertices,
              const MultiviewTrackList<int>& tracks,
              const std::map<FeatureIndex, int>& lookup);

    int count() const;
    void join(int u, int v);
    bool together(int u, int v) const;
    bool compatible(int u, int v) const;

    const Set& find(int v) const;
    T& property(int v);
    const T& property(int v) const;

    typedef typename std::map<int, Set>::const_iterator const_iterator;
    const_iterator begin() const;
    const_iterator end() const;

  private:
    Set& get(int v);

    typedef std::map<int, Set> SetList;
    typedef std::vector<int> FeatureList;

    FeatureList features_;
    SetList sets_;
};

#include "feature_sets.inl"

#endif
