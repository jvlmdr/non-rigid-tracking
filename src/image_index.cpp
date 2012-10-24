#include "image_index.hpp"

ImageIndex::ImageIndex() : view(-1), time(-1) {}

ImageIndex::ImageIndex(int view, int time) : view(view), time(time) {}

bool ImageIndex::operator<(const ImageIndex& other) const {
  return view < other.view || (view == other.view && time < other.time);
}

bool ImageIndex::operator>(const ImageIndex& other) const {
  return other < *this;
}

bool ImageIndex::operator>=(const ImageIndex& other) const {
  return !(*this < other);
}

bool ImageIndex::operator<=(const ImageIndex& other) const {
  return !(other < *this);
}

bool ImageIndex::operator==(const ImageIndex& other) const {
  return view == other.view && time == other.time;
}

bool ImageIndex::operator!=(const ImageIndex& other) const {
  return !(*this == other);
}

std::ostream& operator<<(std::ostream& stream, const ImageIndex& frame) {
  return stream << "(" << frame.view << ", " << frame.time << ")";
}
