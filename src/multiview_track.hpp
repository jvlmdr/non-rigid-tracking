#ifndef MULTIVIEW_TRACK_HPP_
#define MULTIVIEW_TRACK_HPP_

#include "track.hpp"
#include "smart_vector.hpp"

// A frame is identified by a video stream and a time instant.
struct Frame {
  int view;
  int time;

  Frame();
  Frame(int view, int time);

  // Defines an ordering over frame indices.
  bool operator<(const Frame& other) const;
};

// Describes observations of a feature in multiple views at multiple times.
// Assumes the number of views is known but the number of frames is unknown.
template<class T>
class MultiviewTrack {
  public:
    typedef typename SmartVector<Track<T> >::iterator iterator;
    typedef typename SmartVector<Track<T> >::const_iterator const_iterator;

    explicit MultiviewTrack(int num_views);
    MultiviewTrack();

    void reset(int num_views);
    void swap(MultiviewTrack<T>& other);

    const T* get(const Frame& frame) const;
    T* get(const Frame& frame);
    void set(const Frame& frame, const T& x);
    void setTrack(int view, const Track<T>& track);

    // Read access.
    const Track<T>& view(int view) const;
    Track<T>& view(int view);

    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;

    int numViews() const;
    int numFrames() const;
    bool empty() const;

    // Returns the number of views in which the feature was present.
    int numViewsPresent() const;

  private:
    SmartVector<Track<T> > view_tracks_;
    int num_frames_;
};

#include "multiview_track.inl"

#endif
