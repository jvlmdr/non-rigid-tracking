#ifndef TRACK_LIST_HPP_
#define TRACK_LIST_HPP_

#include <deque>
#include "track.hpp"
#include <map>

// List of features which each has some value at a small subset of frames.
//
// Provides:
// O(1) lookup by feature index.
// O(log n) lookup by frame index.
// Ability to iterate through frames in order.
//
// TODO: Advantageous to enforce that tracks are ordered by their first frame?
template<class T>
class TrackList {
  private:
    typedef std::deque<Track<T> > Container;

  public:
    // Returns the first frame in the track.
    int findFirstFrame() const;

    // Returns the number of points in all tracks.
    int countPoints() const;

    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;
    // For std::back_inserter and similar.
    typedef Track<T>& reference;
    typedef const Track<T>& const_reference;

    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;

    TrackList();
    TrackList(int size);

    Track<T>& operator[](int n);
    const Track<T>& operator[](int n) const;

    void push_back(const Track<T>& x);
    Track<T>& back();
    Track<T>& front();
    const Track<T>& back() const;
    const Track<T>& front() const;

    int size() const;
    bool empty() const;
    void clear();
    void swap(TrackList<T>& other);

    int numImageFeatures() const;

  private:
    Container list_;
};

template<class T>
void swap(TrackList<T>& lhs, TrackList<T>& rhs);

////////////////////////////////////////////////////////////////////////////////

// Iterates through a list of tracks one frame at a time.
template<class T>
class TrackListTimeIterator {
  public:
    // Initializes at start of tracks.
    TrackListTimeIterator(const TrackList<T>& tracks);
    // Initializes at a given frame.
    TrackListTimeIterator(const TrackList<T>& tracks, int t);
    // Copy constructor.
    TrackListTimeIterator(const TrackListTimeIterator& rhs);

    typedef std::map<int, T> Points;

    // Writes out the points in the current frame.
    void getPoints(Points& points) const;
    // Returns true if this is the last frame.
    bool end() const;

    // Pre-increment.
    TrackListTimeIterator<T>& operator++();
    // Post-increment.
    TrackListTimeIterator<T> operator++(int);

    // Seek to the start of the tracks.
    void seekToStart();
    // Get current time.
    int t() const;

  private:
    // Maintain a list of positions in each track.
    typedef TrackIterator<T> Cursor;
    typedef std::map<int, Cursor> CursorList;
    CursorList cursors_;
    // Index of current frame.
    int t_;

    // Updates the list of points in this frame and advances the cursor.
    void advance();
};

#include "track_list.inl"

#endif
