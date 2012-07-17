#ifndef TRACK_HPP_
#define TRACK_HPP_

#include <opencv2/core/core.hpp>
#include <map>

template<class T>
class Track_ {
  private:
    typedef std::map<int, T> Map;

  public:
    typedef typename Map::iterator iterator;
    typedef typename Map::const_iterator const_iterator;
    typedef typename Map::reverse_iterator reverse_iterator;
    typedef typename Map::const_reverse_iterator const_reverse_iterator;

    Track_();

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

// A track is a set of (t, x) pairs.
typedef Track_<cv::Point2d> Track;

////////////////////////////////////////////////////////////////////////////////

// A TrackCursor is an iterator that can check if it has reached the end.
// Don't C++ iterators have the ability to do this? Not for lists?
template<class T>
struct TrackCursor_ {
  typedef typename Track_<T>::const_iterator Position;

  const Track_<T>* track;
  Position point;

  TrackCursor_();
  TrackCursor_(const Track_<T>& track, const Position& point);

  bool end() const;

  // Returns a cursor at the start of the track.
  static TrackCursor_ make(const Track_<T>& track);
};

typedef TrackCursor_<cv::Point2d> TrackCursor;

#include "track.inl"

#endif
