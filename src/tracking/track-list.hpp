#ifndef TRACK_LIST_HPP_
#define TRACK_LIST_HPP_

#include "tracking/using.hpp"
#include "tracking/track-list.pb.h"

namespace tracking {

// Describes a list of tracks.
class TrackList {
  public:
    typedef TrackListData::Frame Frame;
    // Iterator types.
    typedef RepeatedPtrField<Frame>::const_iterator const_iterator;
    typedef RepeatedPtrField<Frame>::iterator iterator;

    TrackList();

    inline const_iterator begin() const;
    inline const_iterator end() const;
    inline iterator begin();
    inline iterator end();

    inline int numFrames() const;
    inline int numPoints() const;

  private:
    TrackListData data_;
    int num_points_;
};

}

#include "track-list.inl"

#endif
