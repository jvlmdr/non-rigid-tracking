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

    typedef typename Map::reference reference;
    typedef typename Map::const_reference const_reference;
    typedef typename Map::pointer pointer;
    typedef typename Map::const_pointer const_pointer;

    typedef typename Map::value_type value_type;

    Track();

    T& operator[](int x);
    std::pair<iterator, bool> insert(const value_type& x);
    iterator insert(iterator position, const value_type& x);
    template<class InputIterator>
    void insert(InputIterator first, InputIterator last);

    const_iterator find(int x) const;
    iterator find(int x);

    int size() const;
    bool empty() const;
    void clear();
    void swap(Track<T>& other);

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

template<class T>
void swap(Track<T>& lhs, Track<T>& rhs);

template<class T>
int addTrackSize(int x, const Track<T>& track);

////////////////////////////////////////////////////////////////////////////////

// Iterates through a track.
template<class T>
class TrackIterator {
  public:
    TrackIterator();
    explicit TrackIterator(const Track<T>& track);
    TrackIterator(const TrackIterator<T>& other);

    void next();
    void previous();

    bool end() const;
    bool begin() const;

    const T& get() const;
    int time() const;

  private:
    typedef typename Track<T>::const_iterator Position;

    const Track<T>* track_;
    Position position_;
};

#include "track.inl"

#endif
