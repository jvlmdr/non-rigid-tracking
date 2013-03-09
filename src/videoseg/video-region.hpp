#ifndef VIDEOSEG_VIDEO_REGION_HPP_
#define VIDEOSEG_VIDEO_REGION_HPP_

#include "videoseg/using.hpp"
#include "videoseg/region.hpp"

namespace videoseg {

// Analogous to Matthias' Rasterization3D, except without shared_ptrs.
class VideoRegion {
  public:
    typedef map<int, Rasterization> FrameList;

    inline const FrameList& frames() const;
    inline FrameList& frames();

  private:
    // Frames in which segment is present.
    FrameList frames_;
};

}

#include "videoseg/video-region.inl"

#endif
