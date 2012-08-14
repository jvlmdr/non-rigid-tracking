#ifndef TRACK_LIST_WRITER_HPP_
#define TRACK_LIST_WRITER_HPP_

#include "track_list.hpp"
#include "writer.hpp"

template<class T>
class TrackListWriter : public Writer<TrackList<T> > {
  public:
    TrackListWriter(Writer<Track<T> >& writer);
    ~TrackListWriter();
    void write(cv::FileStorage& file, const TrackList<T>& tracks);

  private:
    Writer<Track<T> >* writer_;
};

template<class T>
bool saveTrackList(const std::string& filename,
                   const TrackList<T>& tracks,
                   Writer<T>& writer);

#include "track_list_writer.inl"

#endif
