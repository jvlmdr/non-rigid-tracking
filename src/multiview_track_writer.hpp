#ifndef MULTIVIEW_TRACK_WRITER_HPP_
#define MULTIVIEW_TRACK_WRITER_HPP_

#include "multiview_track.hpp"
#include "writer.hpp"

template<class T>
class MultiviewTrackWriter : public Writer<MultiviewTrack<T> > {
  public:
    MultiviewTrackWriter(Writer<Track<T> >& writer);
    ~MultiviewTrackWriter();
    void write(cv::FileStorage& file, const MultiviewTrack<T>& track);

  private:
    Writer<Track<T> >* writer_;
};

#include "multiview_track_writer.inl"

#endif
