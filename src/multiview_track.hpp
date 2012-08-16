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

// Assume the number of views is known but the number of frames is unknown.
template<class T>
class MultiviewTrack {
  public:
    MultiviewTrack();
    MultiviewTrack(int num_views);
    void reset(int num_views);
    void swap(MultiviewTrack<T>& other);

    void add(const Frame& frame, const T& x);

    // Provides read access to the track in one view.
    const Track<T>& track(int view) const;
    const std::vector<Track<T> >& tracks() const;

    int numViews() const;
    int numFrames() const;

  private:
    std::vector<Track<T> > tracks_;
    int num_frames_;
};

#include "multiview_track.inl"

#endif
