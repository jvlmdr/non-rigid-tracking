#ifndef MULTIVIEW_TRACK_LIST_READER_HPP_
#define MULTIVIEW_TRACK_LIST_READER_HPP_

#include "multiview_track_list.hpp"
#include "reader.hpp"

template<class T>
class MultiviewTrackListReader : public Reader<MultiviewTrackList<T> > {
  public:
    MultiviewTrackListReader(Reader<MultiviewTrack<T> >& reader);
    ~MultiviewTrackListReader();
    void read(const cv::FileNode& node, MultiviewTrackList<T>& tracks);

  private:
    Reader<MultiviewTrack<T> >* reader_;
};

template<class T>
bool loadMultiviewTrackList(const std::string& filename,
                            MultiviewTrackList<T>& tracks,
                            Reader<T>& reader);

#include "multiview_track_list_reader.inl"

#endif
