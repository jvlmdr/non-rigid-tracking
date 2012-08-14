#ifndef TRACK_LIST_HPP_
#define TRACK_LIST_HPP_

#include <vector>
#include <list>
#include <map>
#include <string>
#include "track.hpp"

// List of features which each has some value at a small subset of frames.
//
// Provides:
// O(1) lookup by feature index.
// O(log n) lookup by frame index.
// Ability to iterate through frames in order.
//
// TODO: Advantageous to enforce that tracks are ordered by their first frame?
template<class T>
class TrackList_ {
  private:
    typedef Track_<T> Track;
    typedef std::vector<Track> List;

  public:
    // Returns the first frame in the track.
    int findFirstFrame() const;

    // Returns the number of points in all tracks.
    int countPoints() const;

    typedef typename List::iterator iterator;
    typedef typename List::const_iterator const_iterator;
    typedef typename List::reverse_iterator reverse_iterator;
    typedef typename List::const_reverse_iterator const_reverse_iterator;

    typedef typename List::reference reference;
    typedef typename List::const_reference const_reference;
    typedef typename List::pointer pointer;
    typedef typename List::const_pointer const_pointer;

    TrackList_();
    TrackList_(int size);

    Track& operator[](int n);
    const Track& operator[](int n) const;

    void push_back(const Track& x);
    Track& back();
    const Track& back() const;
    Track& front();
    const Track& front() const;

    int size() const;
    bool empty() const;
    void clear();
    void swap(TrackList_<T>& other);

    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

    reverse_iterator rbegin();
    const_reverse_iterator rbegin() const;
    reverse_iterator rend();
    const_reverse_iterator rend() const;

  private:
    List list_;
};

////////////////////////////////////////////////////////////////////////////////

// Iterates through a list of tracks one frame at a time.
template<class T>
class FrameIterator_ {
  public:
    typedef std::map<int, T> Points;

    // Writes out the points in the current frame.
    void getPoints(Points& points) const;
    // Returns true if this is the last frame.
    bool end() const;

    // Initializes at start of tracks.
    FrameIterator_(const TrackList_<T>& tracks);
    // Copy constructor.
    FrameIterator_(const FrameIterator_& rhs);

    // Pre-increment.
    FrameIterator_<T>& operator++();
    // Post-increment.
    FrameIterator_<T> operator++(int);

    // Seek to the start of the tracks.
    void seekToStart();
    // Get current time.
    int t() const;

  private:
    // Maintain a list of positions in each track.
    typedef TrackCursor_<T> Cursor;
    typedef std::map<int, Cursor> CursorList;
    CursorList cursors_;
    // Index of current frame.
    int t_;

    // Updates the list of points in this frame and advances the cursor.
    void advance();
};

#include "track_list.inl"

#endif
