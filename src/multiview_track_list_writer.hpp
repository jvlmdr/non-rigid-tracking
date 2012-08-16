#ifndef MULTIVIEW_TRACK_LIST_WRITER_HPP_
#define MULTIVIEW_TRACK_LIST_WRITER_HPP_

#include "multiview_track_list.hpp"
#include "writer.hpp"

template<class T>
class MultiviewTrackListWriter : public Writer<MultiviewTrackListWriter<T> > {
  public:
    MultiviewTrackListWriter(Writer<MultiviewTrackWriter<T> >& writer);
    ~MultiviewTrackListWriter();
    void write(cv::FileStorage& file, const MultiviewTrackList<T>& tracks);

  private:
    Writer<MultiviewTrackWriter<T> >* writer_;
};

template<class T>
bool saveMultiviewTrackList(const std::string& filename,
                            const MultiviewTrackList<T>& tracks,
                            Writer<T>& writer);

#include "multiview_track_list_writer.inl"

#endif
