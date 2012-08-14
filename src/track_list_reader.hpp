#ifndef TRACK_LIST_READER_HPP_
#define TRACK_LIST_READER_HPP_

#include "track_list.hpp"
#include "reader.hpp"

template<class T>
class TrackListReader : public Reader<TrackList_<T> > {
  public:
    TrackListReader(Reader<Track_<T> >& reader);
    ~TrackListReader();
    void read(const cv::FileNode& node, TrackList_<T>& tracks);

  private:
    Reader<Track_<T> >* reader_;
};

template<class T>
bool loadTrackList(const std::string& filename,
                   TrackList_<T>& tracks,
                   Reader<T>& reader);

#include "track_list_reader.inl"

#endif
