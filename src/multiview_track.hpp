#ifndef MULTIVIEW_TRACK_HPP_
#define MULTIVIEW_TRACK_HPP_

#include "track.hpp"
#include <vector>

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
    typedef typename std::vector<Track<T> >::iterator iterator;
    typedef typename std::vector<Track<T> >::const_iterator const_iterator;

    explicit MultiviewTrack(int num_views);
    MultiviewTrack();

    void swap(MultiviewTrack<T>& other);

    const T* point(const Frame& frame) const;
    T* point(const Frame& frame);

    const Track<T>& view(int view) const;
    Track<T>& view(int view);

    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;

    int numViews() const;
    int numFrames() const;
    bool empty() const;

    // Returns the number of image features in all views.
    int numImageFeatures() const;

    // Returns the number of views in which the feature was present.
    int numViewsPresent() const;

  private:
    std::vector<Track<T> > view_tracks_;
    int num_frames_;
};

template<class T>
void swap(MultiviewTrack<T>& lhs, MultiviewTrack<T>& rhs);

template<class T>
int addMultiviewTrackImageFeatures(int x, const MultiviewTrack<T>& track);

#include "multiview_track.inl"

#endif
