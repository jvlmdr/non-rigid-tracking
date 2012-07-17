#ifndef TRACK_LIST_HPP_
#define TRACK_LIST_HPP_

#include <vector>
#include <list>
#include <map>
#include <string>
#include <boost/function.hpp>
#include <opencv2/core/core.hpp>
#include "track.hpp"

template<class T>
class Write {
  public:
    virtual ~Write() = 0;
    virtual void operator()(cv::FileStorage& file, const T& x) = 0;
};

template<class T>
class Read {
  public:
    virtual ~Read() = 0;
    virtual void operator()(const cv::FileNode& node, T& x) = 0;
};

template<class T>
class TrackList_ {
  private:
    typedef Track_<T> Track;
    typedef std::vector<Track> List;

  public:
    // Returns the first frame in the track.
    int findFirstFrame() const;

    // Saves a list of tracks to a file.
    bool save(const std::string& filename, Write<T>& write_point) const;
    // Loads a list of tracks from a file.
    bool load(const std::string& filename, Read<T>& read_point);

    typedef typename List::iterator iterator;
    typedef typename List::const_iterator const_iterator;
    typedef typename List::reverse_iterator reverse_iterator;
    typedef typename List::const_reverse_iterator const_reverse_iterator;

    typedef typename List::reference reference;
    typedef typename List::const_reference const_reference;
    typedef typename List::pointer pointer;
    typedef typename List::const_pointer const_pointer;

    TrackList_();
    TrackList_(int size);

    Track& operator[](int n);
    const Track& operator[](int n) const;

    void push_back(const Track& x);
    Track& back();
    const Track& back() const;
    Track& front();
    const Track& front() const;

    int size() const;
    bool empty() const;
    void clear();
    void swap(TrackList_<T>& other);

    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

    reverse_iterator rbegin();
    const_reverse_iterator rbegin() const;
    reverse_iterator rend();
    const_reverse_iterator rend() const;

  private:
    List list_;
};

// Describes a collection of tracks from a single video sequence.
typedef TrackList_<cv::Point2d> TrackList;

// Saves a list of tracks to a file.
bool saveTracks(const std::string& filename,
                const cv::Size& size,
                const TrackList& tracks);

// Loads a list of tracks from a file.
bool loadTracks(const std::string& filename,
                cv::Size& size,
                TrackList& tracks,
                int* num_tracks);

// Returns the first frame in any track.
int findFirstFrame(const TrackList& tracks);

////////////////////////////////////////////////////////////////////////////////

// Iterates through a list of tracks one frame at a time.
template<class T>
class FrameIterator_ {
  public:
    typedef std::map<int, T> Points;

    // Writes out the points in the current frame.
    void getPoints(Points& points) const;
    // Returns true if this is the last frame.
    bool end() const;

    // Initializes at start of tracks.
    FrameIterator_(const TrackList_<T>& tracks);
    // Copy constructor.
    FrameIterator_(const FrameIterator_& rhs);

    // Pre-increment.
    FrameIterator_<T>& operator++();
    // Post-increment.
    FrameIterator_<T> operator++(int);

  private:
    // Maintain a list of positions in each track.
    typedef TrackCursor_<T> Cursor;
    typedef std::map<int, Cursor> CursorList;
    CursorList cursors_;
    // Index of current frame.
    int t_;

    // Updates the list of points in this frame and advances the cursor.
    void advance();
};

typedef FrameIterator_<cv::Point2d> FrameIterator;

class WritePoint : public Write<cv::Point2d> {
  public:
    ~WritePoint();
    void operator()(cv::FileStorage& file, const cv::Point2d& x);
};

class ReadPoint : public Read<cv::Point2d> {
  public:
    ~ReadPoint();
    void operator()(const cv::FileNode& node, cv::Point2d& x);
};

#include "track_list.inl"

#endif
