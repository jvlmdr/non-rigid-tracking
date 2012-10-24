#ifndef ITERATOR_WRITER_HPP_
#define ITERATOR_WRITER_HPP_

#include "writer.hpp"
#include <vector>
#include <deque>

template<class T, class InputIterator>
void writeSequence(cv::FileStorage& file,
                   Writer<T>& writer,
                   InputIterator begin,
                   InputIterator end);

////////////////////////////////////////////////////////////////////////////////

template<class T, class Container>
class ContainerWriter : public Writer<Container> {
  public:
    ContainerWriter(Writer<T>& writer);
    ~ContainerWriter();
    void write(cv::FileStorage& file, const Container& x);

  private:
    Writer<T>* writer_;
};

template<class T, class Container>
bool saveList(const std::string& filename,
              const Container& list,
              Writer<T>& writer);

////////////////////////////////////////////////////////////////////////////////

// typedef ContainerWriter<T, std::vector<T> > VectorWriter<T>;
template<class T>
class VectorWriter : public Writer<std::vector<T> > {
  public:
    VectorWriter(Writer<T>& writer);
    ~VectorWriter();
    void write(cv::FileStorage& file, const std::vector<T>& x);

  private:
    ContainerWriter<T, std::vector<T> > writer_;
};

// typedef ContainerWriter<T, std::deque<T> > DequeWriter<T>;
template<class T>
class DequeWriter : public Writer<std::deque<T> > {
  public:
    DequeWriter(Writer<T>& writer);
    ~DequeWriter();
    void write(cv::FileStorage& file, const std::deque<T>& x);

  private:
    ContainerWriter<T, std::deque<T> > writer_;
};

#include "iterator_writer.inl"

#endif
