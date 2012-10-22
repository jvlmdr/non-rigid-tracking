#ifndef FEATURE_SETS_HPP_
#define FEATURE_SETS_HPP_

#include <map>
#include <vector>
#include "match_graph.hpp"

class FeatureSets {
  public:
    explicit FeatureSets(const MatchGraph& graph);

    int count() const;
    void join(int u, int v);
    bool together(int u, int v) const;
    bool compatible(int u, int v) const;

    typedef std::map<Frame, int> Set;
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
