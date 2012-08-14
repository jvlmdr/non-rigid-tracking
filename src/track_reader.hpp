#ifndef TRACK_READER_HPP_
#define TRACK_READER_HPP_

#include "track.hpp"
#include "reader.hpp"

template<class T>
class TrackReader : public Reader<Track_<T> > {
  public:
    TrackReader(Reader<T>& reader);
    ~TrackReader();
    void read(const cv::FileNode& node, Track_<T>& track);

  private:
    Reader<T>* reader_;
};

#include "track_reader.inl"

#endif
