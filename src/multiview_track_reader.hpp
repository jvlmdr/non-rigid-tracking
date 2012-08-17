#ifndef MULTIVIEW_TRACK_READER_HPP_
#define MULTIVIEW_TRACK_READER_HPP_

#include "multiview_track.hpp"
#include "reader.hpp"

template<class T>
class MultiviewTrackReader : public Reader<MultiviewTrack<T> > {
  public:
    MultiviewTrackReader(Reader<Track<T> >& reader);
    ~MultiviewTrackReader();
    void read(const cv::FileNode& node, MultiviewTrack<T>& track);

  private:
    Reader<Track<T> >* reader_;
};

#include "multiview_track_reader.inl"

#endif
