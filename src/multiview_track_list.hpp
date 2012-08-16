#ifndef MULTIVIEW_TRACK_LIST_HPP_
#define MULTIVIEW_TRACK_LIST_HPP_

#include "track.hpp"
#include "multiview_track.hpp"
#include <vector>

template<class T>
class MultiviewTrackList {
  public:
    MultiviewTrackList();
    MultiviewTrackList(int num_views);
    void reset(int num_views);
    void swap(MultiviewTrackList<T>& other);

    // Swaps a multi-view track into the list.
    void add(MultiviewTrack<T>& track);

    // Provides read access to the tracks in one view.
    const MultiviewTrack<T>& track(int id) const;
    const std::vector<MultiviewTrack<T> >& tracks() const;

    int numTracks() const;
    int numViews() const;
    int numFrames() const;

  private:
    // Store as a list of multiview tracks.
    // This seems like the most natural way to add() to the data structure.
    std::vector<MultiviewTrack<T> > tracks_;
    int num_views_;
    int num_frames_;
};

#include "multiview_track_list.inl"

#endif
