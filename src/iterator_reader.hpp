#ifndef ITERATOR_READER_HPP_
#define ITERATOR_READER_HPP_

#include "reader.hpp"
#include <vector>
#include <deque>

template<class T, class Container>
class ContainerReader : public Reader<Container> {
  public:
    ContainerReader(Reader<T>& reader);
    ~ContainerReader();
    bool read(const cv::FileNode& node, Container& list);

  private:
    Reader<T>* reader_;
};

template<class T, class Container>
bool loadList(const std::string& file, Reader<T>& reader, Container list);

// typedef ContainerReader<T, std::vector<T> > VectorReader<T>;
template<class T>
class VectorReader : public Reader<std::vector<T> > {
  public:
    VectorReader(Reader<T>& reader);
    ~VectorReader();
    bool read(const cv::FileNode& node, std::vector<T>& list);

  private:
    ContainerReader<T, std::vector<T> > reader_;
};

// typedef ContainerReader<T, std::deque<T> > DequeReader<T>;
template<class T>
class DequeReader : public Reader<std::deque<T> > {
  public:
    DequeReader(Reader<T>& reader);
    ~DequeReader();
    bool read(const cv::FileNode& node, std::deque<T>& list);

  private:
    ContainerReader<T, std::deque<T> > reader_;
};

// Where the magic happens.
template<class T, class OutputIterator>
bool readSequence(const cv::FileNode& node,
                  Reader<T>& reader,
                  OutputIterator output);

#include "iterator_reader.inl"

#endif
