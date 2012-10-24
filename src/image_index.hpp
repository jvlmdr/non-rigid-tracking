#include <ostream>

// An image is identified by its view and time instant.
struct ImageIndex {
  int view;
  int time;

  ImageIndex();
  ImageIndex(int view, int time);

  // Defines an ordering over frame indices.
  bool operator<(const ImageIndex& other) const;
  bool operator>(const ImageIndex& other) const;
  bool operator>=(const ImageIndex& other) const;
  bool operator<=(const ImageIndex& other) const;

  bool operator==(const ImageIndex& other) const;
  bool operator!=(const ImageIndex& other) const;
};

std::ostream& operator<<(std::ostream& stream, const ImageIndex& frame);
