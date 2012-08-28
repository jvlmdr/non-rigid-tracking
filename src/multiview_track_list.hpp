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
    typedef std::vector<MultiviewTrack<T> > List;

    explicit MultiviewTrackList(int num_views);
    MultiviewTrackList();
    MultiviewTrackList(const MultiviewTrackList<T>& other);

    void reset(int num_views);
    void swap(MultiviewTrackList<T>& other);

    // Swaps a multi-view track into the list.
    void add(MultiviewTrack<T>& track);

    // Provides read access to the tracks in one view.
    const MultiviewTrack<T>& track(int id) const;
    const List& tracks() const;

    int numTracks() const;
    int numViews() const;
    int numFrames() const;

  private:
    // Store as a list of multiview tracks.
    // This seems like the most natural way to add() to the data structure.
    List tracks_;
    int num_views_;
    int num_frames_;
};

// Iterates over all tracks in one view of a MultiviewTrackList in time order.
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
    typedef TrackIterator<T> Cursor;
    typedef std::vector<Cursor> CursorList;

    int view_;
    int time_;
    CursorList cursors_;
};

// Iterates over all tracks in a MultiviewTrackList in time order.
template<class T>
class MultiViewTimeIterator {
  public:
    MultiViewTimeIterator();
    MultiViewTimeIterator(const MultiviewTrackList<T>& tracks);

    // Advance to next time instant.
    void next();
    // Reached the end of the tracks?
    bool end() const;
    // Current time index.
    int time() const;
    // Populates a map of feature -> (a map of view -> point).
    void get(std::map<int, std::map<int, T> >& points) const;
    // Populates a map of feature -> point for a single view.
    void getView(int view, std::map<int, T>& points) const;

    const SingleViewTimeIterator<T>& view(int i) const;

  private:
    typedef std::vector<SingleViewTimeIterator<T> > IteratorList;

    IteratorList views_;
    int time_;
};

#include "multiview_track_list.inl"

#endif
