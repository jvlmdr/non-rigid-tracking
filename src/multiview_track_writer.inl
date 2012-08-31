#include "iterator_writer.hpp"

template<class T>
MultiviewTrackWriter<T>::MultiviewTrackWriter(Writer<Track<T> >& writer)
    : writer_(&writer) {}

template<class T>
MultiviewTrackWriter<T>::~MultiviewTrackWriter() {}

template<class T>
void MultiviewTrackWriter<T>::write(cv::FileStorage& file,
                                    const MultiviewTrack<T>& track) {
  IteratorWriter<Track<T>, MultiviewTrack<T> > list_writer(*writer_);
  list_writer.write(file, track);
}
