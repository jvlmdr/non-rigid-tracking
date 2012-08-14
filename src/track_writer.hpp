#ifndef TRACK_WRITER_HPP_
#define TRACK_WRITER_HPP_

#include "track.hpp"
#include "writer.hpp"

template<class T>
class TrackWriter : public Writer<Track<T> > {
  public:
    TrackWriter(Writer<T>& writer);
    ~TrackWriter();
    void write(cv::FileStorage& file, const Track<T>& track);

  private:
    Writer<T>* writer_;
};

#include "track_writer.inl"

#endif
