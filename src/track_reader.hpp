#ifndef TRACK_READER_HPP_
#define TRACK_READER_HPP_

#include "track.hpp"
#include "reader.hpp"

template<class T>
class TrackReader : public Reader<Track<T> > {
  public:
    TrackReader(Reader<T>& reader);
    ~TrackReader();
    bool read(const cv::FileNode& node, Track<T>& track);

  private:
    Reader<T>* reader_;
};

#include "track_reader.inl"

#endif
