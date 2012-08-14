#ifndef TRACK_LIST_WRITER_HPP_
#define TRACK_LIST_WRITER_HPP_

#include "track_list.hpp"
#include "writer.hpp"

template<class T>
class TrackListWriter : public Writer<TrackList_<T> > {
  public:
    TrackListWriter(Writer<Track_<T> >& writer);
    ~TrackListWriter();
    void write(cv::FileStorage& file, const TrackList_<T>& tracks);

  private:
    Writer<Track_<T> >* writer_;
};

template<class T>
bool saveTrackList(const std::string& filename,
                   const TrackList_<T>& tracks,
                   Writer<T>& writer);

#include "track_list_writer.inl"

#endif
