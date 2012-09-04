#ifndef TRACK_LIST_READER_HPP_
#define TRACK_LIST_READER_HPP_

#include "track_list.hpp"
#include "reader.hpp"

template<class T>
class TrackListReader : public Reader<TrackList<T> > {
  public:
    TrackListReader(Reader<T>& reader);
    ~TrackListReader();
    bool read(const cv::FileNode& node, TrackList<T>& tracks);

  private:
    Reader<T>* reader_;
};

template<class T>
bool loadTrackList(const std::string& filename,
                   TrackList<T>& tracks,
                   Reader<T>& reader);

#include "track_list_reader.inl"

#endif
