#include <list>
#include <numeric>
#include <string>
#include <cstdlib>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "random.hpp"
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"
#include "track_list.hpp"
#include "hsv.hpp"
#include "sift_position.hpp"
#include "draw_sift_position.hpp"

#include "multiview_track_list_reader.hpp"
#include "track_list_reader.hpp"
#include "default_reader.hpp"
#include "read_lines.hpp"
#include "sift_position_reader.hpp"
#include "read_image.hpp"

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const double LINE_THICKNESS = 2;

const int PIXELS_PER_TICK = 6;
const int RADIUS = 3;
const int MIN_CLEARANCE = 6;
const int MAX_NUM_TRACKS = 16;
const int MAX_NUM_VIDEOS = 16;
const cv::Scalar TIME_LINE_COLOR(100, 100, 100);
//const int MIN_TIME_CLEARANCE = 4;

DEFINE_string(output_format, "%d-%d.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

DEFINE_bool(show_matches, false, "Show cross-view matches?");
DEFINE_bool(exclude_single_view, true, "Show multi-view tracks that aren't?");
DEFINE_bool(show_trails, false, "Show point trails within view?");

class FileStream {
  public:
    virtual ~FileStream() {}
    virtual std::string get(int t) const = 0;
};

class SingleViewFileStream : public FileStream {
  public:
    SingleViewFileStream();
    SingleViewFileStream(const std::string& format);
    ~SingleViewFileStream();
    std::string get(int t) const;

  private:
    std::string format_;
};

class MultiViewFileStream : public FileStream {
  public:
    MultiViewFileStream();
    MultiViewFileStream(const std::string& format, const std::string& view);
    ~MultiViewFileStream();
    std::string get(int t) const;

  private:
    std::string format_;
    std::string view_;
};

SingleViewFileStream::SingleViewFileStream() : format_() {}

SingleViewFileStream::SingleViewFileStream(const std::string& format)
    : format_(format) {}

SingleViewFileStream::~SingleViewFileStream() {}

std::string SingleViewFileStream::get(int t) const {
  return boost::str(boost::format(format_) % (t + 1));
}

MultiViewFileStream::MultiViewFileStream() : format_(), view_() {}

MultiViewFileStream::MultiViewFileStream(const std::string& format,
                                         const std::string& view)
    : format_(format), view_(view) {}

MultiViewFileStream::~MultiViewFileStream() {}

std::string MultiViewFileStream::get(int t) const {
  return boost::str(boost::format(format_) % view_ % (t + 1));
}

////////////////////////////////////////////////////////////////////////////////

void trackListToMultiview(const TrackList<SiftPosition>& single,
                          MultiviewTrackList<SiftPosition>& multi) {
  multi = MultiviewTrackList<SiftPosition>(1);

  TrackList<SiftPosition>::const_iterator track;
  for (track = single.begin(); track != single.end(); ++track) {
    multi.push_back(MultiviewTrack<SiftPosition>(1));
    multi.back().view(0) = *track;
  }
}

////////////////////////////////////////////////////////////////////////////////

struct ViewCollage {
  // These cv::Mats are simply headers referring to external data.
  cv::Mat xy;
  cv::Mat xt;
  cv::Mat ty;
  cv::Mat tt;

  ViewCollage(const cv::Mat& image,
              cv::Size size,
              int num_frames,
              int pixels_per_tick) {
    int width = size.width;
    int height = size.height;
    int depth = num_frames * pixels_per_tick;

    xy = image(cv::Rect(cv::Point(    0,      0), cv::Size(width, height)));
    xt = image(cv::Rect(cv::Point(    0, height), cv::Size(width,  depth)));
    ty = image(cv::Rect(cv::Point(width,      0), cv::Size(depth, height)));
    tt = image(cv::Rect(cv::Point(width, height), cv::Size(depth,  depth)));
  }
};

struct Collage {
  // This cv::Mat owns the image data.
  cv::Mat image;
  std::vector<ViewCollage> views;

  void init(cv::Size size,
            int num_views,
            int num_frames,
            int pixels_per_tick) {
    int image_width = size.width;
    int image_height = size.height;
    int depth = num_frames * pixels_per_tick;

    // View consists of image and xt- and ty-projections.
    int view_width = image_width + depth;
    int view_height = image_height + depth;
    cv::Size view_size(view_width, view_height);

    // Stack views side by side.
    int collage_width = num_views * view_width;
    int collage_height = view_height;
    cv::Size collage_size(collage_width, collage_height);

    // Create image and set to black.
    image.create(collage_size, cv::DataType<cv::Vec3b>::type);
    image = cv::Scalar::all(0);

    // Create each view.
    views.clear();
    for (int i = 0; i < num_views; i += 1) {
      cv::Point offset(i * view_width, 0);
      cv::Mat view_image = image(cv::Rect(offset, view_size));

      ViewCollage view_collage(view_image, size, num_frames, pixels_per_tick);
      views.push_back(view_collage);
    }
  }
};

int numObservations(const MultiviewTrack<SiftPosition>& track) {
  // Add size of all tracks.
  return std::accumulate(track.begin(), track.end(), 0,
      addTrackSize<SiftPosition>);
}

bool hasMoreObservations(const MultiviewTrack<SiftPosition>& lhs,
                         const MultiviewTrack<SiftPosition>& rhs) {
  return numObservations(lhs) > numObservations(rhs);
}

void removeSingleViewTracks(const MultiviewTrackList<SiftPosition>& input,
                            MultiviewTrackList<SiftPosition>& output) {
  output = MultiviewTrackList<SiftPosition>(input.numViews());

  // Copy track list.
  typedef std::list<MultiviewTrack<SiftPosition> > TrackList;
  TrackList tracks(input.begin(), input.end());

  TrackList::iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Filter out single-view tracks.
    if (track->numViewsPresent() > 1) {
      output.push_back(MultiviewTrack<SiftPosition>());
      output.back().swap(*track);
    }
  }
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Visualizes single- or multi-view tracks" << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks image-format num-frames" << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks image-format view-names num-frames" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (!(argc == 4 || argc == 5)) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

bool loadImages(const std::vector<const FileStream*>& streams,
                int time,
                int num_frames,
                Collage& collage) {
  int num_views = streams.size();

  for (int i = 0; i < num_views; i += 1) {
    // Load image for this frame.
    cv::Mat image;
    cv::Mat gray_image;
    std::string file = streams[i]->get(time);
    bool ok = readImage(file, image, gray_image);
    if (!ok) {
      return false;
    }

    // Initialize collage at first image.
    if (i == 0) {
      collage.init(image.size(), num_views, num_frames, PIXELS_PER_TICK);
    }

    // Copy into collage.
    cv::Mat viewport = collage.views[i].xy;
    CHECK(viewport.size() == image.size());
    CHECK(viewport.type() == image.type());
    image.copyTo(viewport);
  }

  return true;
}

void drawFeatures(cv::Mat& image,
                  const std::map<int, SiftPosition>& features,
                  const std::vector<cv::Scalar>& colors) {
  typedef std::map<int, SiftPosition> FeatureSet;
  typedef std::vector<cv::Scalar> ColorList;

  FeatureSet::const_iterator mapping;
  for (mapping = features.begin(); mapping != features.end(); ++mapping) {
    int index = mapping->first;
    const SiftPosition& feature = mapping->second;

    drawSiftPosition(image, feature, colors[index], LINE_THICKNESS);
  }
}

void getFeaturesInView(
    const std::map<int, std::map<int, SiftPosition> >& features,
    int view,
    std::map<int, SiftPosition>& subset) {
  subset.clear();

  // Iterate through multiview observations of each feature.
  std::map<int, std::map<int, SiftPosition> >::const_iterator observations;
  for (observations = features.begin();
       observations != features.end();
       ++observations) {
    int id = observations->first;

    // Check if this feature appeared in the current view.
    std::map<int, SiftPosition>::const_iterator observation;
    observation = observations->second.find(view);

    if (observation != observations->second.end()) {
      // If it was, then add it to the list.
      subset[id] = observation->second;
    }
  }
}

void drawFeaturesInAllViews(
    const std::map<int, std::map<int, SiftPosition> >& features,
    Collage& collage,
    const std::vector<cv::Scalar>& colors) {
  int num_views = collage.views.size();

  // Render features in each view.
  for (int i = 0; i < num_views; i += 1) {
    // Get features for this view.
    std::map<int, SiftPosition> subset;
    getFeaturesInView(features, i, subset);

    drawFeatures(collage.views[i].xy, subset, colors);
  }
}

void moveIntoList(MultiviewTrackList<SiftPosition>& tracks,
                  std::list<MultiviewTrack<SiftPosition> >& list) {
  list.clear();

  MultiviewTrackList<SiftPosition>::iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    list.push_back(MultiviewTrack<SiftPosition>());
    list.back().swap(*track);
  }
}

bool tracksTouch(const Track<SiftPosition>& lhs,
                 const Track<SiftPosition>& rhs,
                 double tolerance) {
  Track<SiftPosition>::const_iterator p = lhs.begin();
  Track<SiftPosition>::const_iterator q = rhs.begin();
  bool touched = false;

  while (p != lhs.end() && q != rhs.end() && !touched) {
    if (p->first < q->first) {
      ++p;
    } else if (q->first < p->first) {
      ++q;
    } else { // p->first == q->first
      // Check clearance.
      if (std::abs(p->second.x - q->second.x) < tolerance) {
        touched = true;
      } else if (std::abs(p->second.y - q->second.y) < tolerance) {
        touched = true;
      }

      // Advance both pointers.
      ++p;
      ++q;
    }
  }

  return touched;
}

bool multiviewTracksTouch(const MultiviewTrack<SiftPosition>& lhs,
                          const MultiviewTrack<SiftPosition>& rhs,
                          double tolerance) {
  CHECK(lhs.numViews() == rhs.numViews());

  // Compare within each view independently.
  MultiviewTrack<SiftPosition>::const_iterator p = lhs.begin();
  MultiviewTrack<SiftPosition>::const_iterator q = rhs.begin();
  bool touched = false;

  while (p != lhs.end() && !touched) {
    touched = tracksTouch(*p, *q, tolerance);
    ++p;
    ++q;
  }

  return touched;
}

bool touchesAnyTrack(const MultiviewTrack<SiftPosition>& query,
                     const MultiviewTrackList<SiftPosition>& existing,
                     double tolerance) {
  MultiviewTrackList<SiftPosition>::const_iterator track;
  for (track = existing.begin(); track != existing.end(); ++track) {
    if (multiviewTracksTouch(query, *track, tolerance)) {
      return true;
    }
  }

  return false;
}

void removeNextSubset(std::list<MultiviewTrack<SiftPosition> >& tracks,
                      MultiviewTrackList<SiftPosition>& subset,
                      double min_clearance,
                      int max_num_tracks) {
  // Assume that all tracks have same number of views.
  subset = MultiviewTrackList<SiftPosition>(tracks.front().numViews());

  std::list<MultiviewTrack<SiftPosition> >::iterator track = tracks.begin();
  while (track != tracks.end() && int(subset.numTracks()) < max_num_tracks) {
    // Check if track satisfies clearance requirements.
    bool compatible = !touchesAnyTrack(*track, subset, min_clearance);

    if (compatible) {
      // Move to subset.
      subset.push_back(MultiviewTrack<SiftPosition>());
      subset.back().swap(*track);

      // Erase from list.
      tracks.erase(track++);
    } else {
      // Try next track.
      ++track;
    }
  }
}

void splitIntoSubsets(std::list<MultiviewTrack<SiftPosition> >& tracks,
                      std::deque<MultiviewTrackList<SiftPosition> >& subsets,
                      double min_clearance,
                      int max_num_tracks,
                      int max_num_subsets) {
  while (!tracks.empty() && int(subsets.size()) < max_num_subsets) {
    MultiviewTrackList<SiftPosition> subset;
    removeNextSubset(tracks, subset, min_clearance, max_num_tracks);

    LOG(INFO) << subset.numTracks();

    subsets.push_back(MultiviewTrackList<SiftPosition>());
    subsets.back().swap(subset);
  }
}

void drawTrackTimeSlices(const Track<SiftPosition>& track,
                         ViewCollage& collage,
                         const cv::Scalar& color,
                         int t) {
  Track<SiftPosition>::const_iterator element;

  cv::Point prev_ty;
  cv::Point prev_xt;

  for (element = track.begin(); element != track.end(); ++element) {
    int u = element->first;
    const SiftPosition& feature = element->second;

    int radius = RADIUS;
    if (t == u) {
      radius *= 2;
    }

    cv::Point center;
    // First draw in ty plot.
    center = cv::Point((u + 0.5) * PIXELS_PER_TICK + 0.5, feature.y);
    cv::circle(collage.ty, center, radius, color, -1);
    if (element != track.begin()) {
      cv::line(collage.ty, prev_ty, center, color, LINE_THICKNESS);
    }
    prev_ty = center;

    // Then in xt plot.
    center = cv::Point(feature.x, (u + 0.5) * PIXELS_PER_TICK + 0.5);
    cv::circle(collage.xt, center, radius, color, -1);
    if (element != track.begin()) {
      cv::line(collage.xt, prev_xt, center, color, LINE_THICKNESS);
    }
    prev_xt = center;
  }
}

void drawTimeSliceLines(ViewCollage& collage, int t) {
  int width = collage.xy.cols;
  int height = collage.xy.rows;
  cv::Scalar color = TIME_LINE_COLOR;

  cv::Point p1;
  cv::Point p2;

  // t vs. y
  p1 = cv::Point((t + 0.5) * PIXELS_PER_TICK + 0.5, 0);
  p2 = cv::Point((t + 0.5) * PIXELS_PER_TICK + 0.5, height - 1);
  cv::line(collage.ty, p1, p2, color, LINE_THICKNESS);

  // x vs. t
  p1 = cv::Point(0, (t + 0.5) * PIXELS_PER_TICK + 0.5);
  p2 = cv::Point(width - 1, (t + 0.5) * PIXELS_PER_TICK + 0.5);
  cv::line(collage.xt, p1, p2, color, LINE_THICKNESS);
}

void drawTimeSliceLinesInAllViews(Collage& collage, int t) {
  int num_views = collage.views.size();

  for (int i = 0; i < num_views; i += 1) {
    drawTimeSliceLines(collage.views[i], t);
  }
}

void drawMultiviewTrackTimeSlices(const MultiviewTrack<SiftPosition>& track,
                                  Collage& collage,
                                  const cv::Scalar& color,
                                  int t) {
  drawTimeSliceLinesInAllViews(collage, t);

  MultiviewTrack<SiftPosition>::const_iterator view;
  int i = 0;

  for (view = track.begin(); view != track.end(); ++view) {
    drawTrackTimeSlices(*view, collage.views[i], color, t);
    i += 1;
  }
}

void drawTimeSlices(const MultiviewTrackList<SiftPosition>& tracks,
                    Collage& collage,
                    const std::vector<cv::Scalar>& colors,
                    int t) {
  CHECK(int(colors.size()) == tracks.numTracks());

  MultiviewTrackList<SiftPosition>::const_iterator track;
  std::vector<cv::Scalar>::const_iterator color = colors.begin();

  for (track = tracks.begin(); track != tracks.end(); ++track) {
    drawMultiviewTrackTimeSlices(*track, collage, *color, t);
    ++color;
  }
}

std::string makeTimeFilename(boost::format format, int time) {
  return boost::str(format % (time + 1));
}

void drawMovie(const MultiviewTrackList<SiftPosition>& tracks,
               const std::vector<cv::Scalar>& colors,
               int num_frames,
               const std::vector<const FileStream*>& image_streams,
               bool show_matches,
               bool show_trails,
               bool display,
               bool save,
               boost::format output_format) {
  // Iterate through time.
  MultiViewTimeIterator<SiftPosition> iterator(tracks);

  for (int t = 0; t < num_frames; t += 1) {
    // Get points at this time instant, indexed by feature number.
    std::map<int, std::map<int, SiftPosition> > features;
    iterator.get(features);

    // Load image for each view.
    Collage collage;
    bool ok = loadImages(image_streams, t, num_frames, collage);
    CHECK(ok) << "Could not load images";

    // Visualize all features.
    drawFeaturesInAllViews(features, collage, colors);

    // Visualize all features.
    drawTimeSlices(tracks, collage, colors, t);

    if (save) {
      std::string output_file = makeTimeFilename(output_format, t);
      ok = cv::imwrite(output_file, collage.image);
      CHECK(ok) << "Could not save image";
    }

    if (display) {
      cv::imshow("tracks", collage.image);
      cv::waitKey(10);
    }

    iterator.next();
  }

  CHECK(iterator.end()) << "Did not reach end of tracks";
}

int main(int argc, char** argv) {
  bool ok;

  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];
  int num_frames;

  MultiviewTrackList<SiftPosition> tracks;

  int num_views;
  std::vector<const FileStream*> image_streams;
  SingleViewFileStream single_view_stream;
  std::list<MultiViewFileStream> multi_view_streams;

  if (argc == 4) {
    // Single view.
    LOG(INFO) << "Single view mode";
    num_frames = boost::lexical_cast<int>(argv[3]);
    num_views = 1;

    // Load tracks.
    TrackList<SiftPosition> single_view_tracks;
    SiftPositionReader feature_reader;
    ok = loadTrackList(tracks_file, single_view_tracks, feature_reader);
    CHECK(ok) << "Could not load tracks";
    LOG(INFO) << "Loaded " << single_view_tracks.size() << " tracks";

    // Convert to multi-view tracks (with one view).
    trackListToMultiview(single_view_tracks, tracks);

    // Set up single-view image stream.
    single_view_stream = SingleViewFileStream(image_format);
    image_streams.push_back(&single_view_stream);
  } else {
    // Multiview.
    LOG(INFO) << "Multiple view mode";
    std::string views_file = argv[3];
    num_frames = boost::lexical_cast<int>(argv[4]);

    // Load tracks.
    SiftPositionReader feature_reader;
    ok = loadMultiviewTrackList(tracks_file, tracks, feature_reader);
    CHECK(ok) << "Could not load tracks";
    LOG(INFO) << "Loaded " << tracks.numTracks() << " multi-view tracks";

    // Load names of views.
    std::vector<std::string> views;
    ok = readLines(views_file, views);
    CHECK(ok) << "Could not load view names";
    num_views = views.size();

    // Ensure that number of views matches.
    CHECK(num_views == tracks.numViews());

    // Set up multi-view image streams.
    std::vector<std::string>::const_iterator view;
    for (view = views.begin(); view != views.end(); ++view) {
      // Create multi-view stream.
      multi_view_streams.push_back(MultiViewFileStream(image_format, *view));
      // Add a pointer to the list of streams.
      image_streams.push_back(&multi_view_streams.back());
    }
  }

  // Ensure that there aren't more frames than there are in the sequence.
  // TODO: Is there any point storing the number of frames?
  CHECK(tracks.numFrames() <= num_frames);

  // Remove any single-view tracks.
  if (num_views > 1 && FLAGS_exclude_single_view) {
    MultiviewTrackList<SiftPosition> multi;
    removeSingleViewTracks(tracks, multi);
    tracks.swap(multi);
  }

  // Sort from most to least observations.
  std::sort(tracks.begin(), tracks.end(), hasMoreObservations);

  // Move into a linked list.
  std::list<MultiviewTrack<SiftPosition> > list;
  moveIntoList(tracks, list);

  // Split into subsets.
  std::deque<MultiviewTrackList<SiftPosition> > subsets;
  splitIntoSubsets(list, subsets, MIN_CLEARANCE, MAX_NUM_TRACKS,
      MAX_NUM_VIDEOS);

  int i = 0;
  std::deque<MultiviewTrackList<SiftPosition> >::const_iterator subset;
  for (subset = subsets.begin(); subset != subsets.end(); ++subset) {
    LOG(INFO) << "Rendering movie " << (i + 1) << " of " << subsets.size();

    // Generate a color for each track.
    std::vector<cv::Scalar> colors;
    evenlySpacedColors(subset->numTracks(), SATURATION, BRIGHTNESS, colors);

    // Draw movie.
    drawMovie(*subset, colors, num_frames, image_streams, FLAGS_show_matches,
        FLAGS_show_trails, FLAGS_display, FLAGS_save,
        boost::format(FLAGS_output_format) % i);

    i += 1;
  }

  return 0;
}
