#ifndef FEATURE_SETS_HPP_
#define FEATURE_SETS_HPP_

#include <map>
#include <vector>
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"
#include "feature_index.hpp"

class FeatureSets {
  public:
    FeatureSets();
    // Add each vertex to its own feature set.
    void init(const std::vector<Frame>& vertices);
    // Create a vertex set for each track.
    // If a vertex does not appear in a track, it gets its own set.
    //
    // Parameters:
    // vertices -- The frame that each vertex was observed in.
    // tracks -- Multiview tracks, containing the index of the feature within
    //   its image.
    // lookup -- A lookup from feature (view, time, id) to vertex index.
    void init(const std::vector<Frame>& vertices,
              const MultiviewTrackList<int>& tracks,
              const std::map<FeatureIndex, int>& lookup);

    int count() const;
    void join(int u, int v);
    bool together(int u, int v) const;
    bool compatible(int u, int v) const;

    struct Set {
      int index;
      std::map<Frame, int> elements;
    };

    const Set& find(int v) const;

    typedef std::map<int, Set>::const_iterator const_iterator;
    const_iterator begin() const;
    const_iterator end() const;

  private:
    //typedef std::list<Set> SetList;
    typedef std::map<int, Set> SetList;
    //typedef std::vector<SetList::iterator> FeatureList;
    typedef std::vector<int> FeatureList;

    FeatureList features_;
    SetList sets_;
};

#endif
