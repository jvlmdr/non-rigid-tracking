#include "multiview_track.hpp"

Frame::Frame() : view(-1), time(-1) {}

Frame::Frame(int view, int time) : view(view), time(time) {}

bool Frame::operator<(const Frame& other) const {
  return (view < other.view) || (view == other.view && time < other.time);
}
