#ifndef TRACK_HPP_
#define TRACK_HPP_

#include <opencv2/core/core.hpp>
#include <map>

template<class T>
class Track {
  private:
    typedef std::map<int, T> Map;

  public:
    typedef typename Map::iterator iterator;
    typedef typename Map::const_iterator const_iterator;
    typedef typename Map::reverse_iterator reverse_iterator;
    typedef typename Map::const_reverse_iterator const_reverse_iterator;

    Track();

    T& operator[](int x);
    const_iterator find(int x) const;
    iterator find(int x);

    int size() const;
    bool empty() const;
    void clear();

    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

    reverse_iterator rbegin();
    const_reverse_iterator rbegin() const;
    reverse_iterator rend();
    const_reverse_iterator rend() const;

  private:
    Map map_;
};

////////////////////////////////////////////////////////////////////////////////

// A TrackCursor is an iterator that can check if it has reached the end.
// Don't C++ iterators have the ability to do this? Not for lists?
template<class T>
struct TrackCursor_ {
  typedef typename Track<T>::const_iterator Position;

  const Track<T>* track;
  Position point;

  TrackCursor_();
  TrackCursor_(const Track<T>& track, const Position& point);

  bool end() const;

  // Returns a cursor at the start of the track.
  static TrackCursor_ make(const Track<T>& track);
};

#include "track.inl"

#endif
