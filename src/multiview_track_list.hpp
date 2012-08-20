#ifndef MULTIVIEW_TRACK_LIST_HPP_
#define MULTIVIEW_TRACK_LIST_HPP_

#include "track.hpp"
#include "multiview_track.hpp"
#include <vector>
#include <list>

// Describes a collection of multi-view tracks.
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

template<class T>
class SingleViewTimeIterator {
  public:
    SingleViewTimeIterator();
    SingleViewTimeIterator(const MultiviewTrackList<T>& tracks, int view);

    // Advance to next time instant.
    void next();
    // Reached the end of the tracks?
    bool end() const;
    // Current time index.
    int time() const;
    // Copies out the subset of points observed in the current frame.
    void get(std::map<int, T>& points) const;

  private:
    const MultiviewTrackList<T>* tracks_;
    int view_;
    int time_;
    typedef typename Track<T>::const_iterator Cursor;
    typedef std::vector<Cursor> CursorList;
    CursorList cursors_;
};

#include "multiview_track_list.inl"

#endif
