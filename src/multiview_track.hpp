#ifndef MULTIVIEW_TRACK_HPP_
#define MULTIVIEW_TRACK_HPP_

#include "track.hpp"
#include "image_index.hpp"
#include <vector>

// Describes observations of a feature in multiple views at multiple times.
// Assumes the number of views is known but the number of frames is unknown.
template<class T>
class MultiviewTrack {
  public:
    typedef typename std::vector<Track<T> >::iterator iterator;
    typedef typename std::vector<Track<T> >::const_iterator const_iterator;

    explicit MultiviewTrack(int num_views);
    MultiviewTrack();

    void swap(MultiviewTrack<T>& other);

    const T* point(const ImageIndex& frame) const;
    T* point(const ImageIndex& frame);

    const Track<T>& view(int view) const;
    Track<T>& view(int view);

    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;

    int numViews() const;
    int numFrames() const;
    bool empty() const;

    // Returns the index of the first and last frame.
    int firstFrameNumber() const;
    int lastFrameNumber() const;

    // Returns the number of image features in all views.
    int numImageFeatures() const;

    // Returns the number of views in which the feature was present.
    int numViewsPresent() const;

    // Iterates through points in a MultiViewTrack in time order.
    class TimeIterator {
      public:
        TimeIterator();
        TimeIterator(const MultiviewTrack<T>& track);

        // Advance to next time instant.
        void next();
        // Reached the end of the tracks?
        bool end() const;
        // Current time index.
        int time() const;
        // Populates a map of view -> point.
        void get(std::map<int, T>& points) const;

      private:
        typedef TrackIterator<T> Cursor;
        typedef std::vector<Cursor> CursorList;

        CursorList cursors_;
        int time_;
    };

    // Iterates over all image features. No guarantees about order.
    class FeatureIterator {
      public:
        FeatureIterator();
        FeatureIterator(const MultiviewTrack<T>& tracks);

        void begin();
        void next();
        bool end() const;
        std::pair<ImageIndex, const T*> get() const;

      private:
        const MultiviewTrack<T>* track_;
        typename MultiviewTrack<T>::const_iterator view_;
        typename Track<T>::const_iterator feature_;
    };

  private:
    typedef std::vector<Track<T> > Container;
    Container view_tracks_;
    int num_frames_;
};

template<class T>
void swap(MultiviewTrack<T>& lhs, MultiviewTrack<T>& rhs);

template<class T>
int addMultiviewTrackImageFeatures(int x, const MultiviewTrack<T>& track);

////////////////////////////////////////////////////////////////////////////////


#include "multiview_track.inl"

#endif
