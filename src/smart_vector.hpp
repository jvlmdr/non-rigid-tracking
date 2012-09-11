#ifndef SMART_VECTOR_HPP_
#define SMART_VECTOR_HPP_

#include <vector>
#include <list>

// This data structure provides:
//  - O(1) amortized appending without copying objects (compare to std::vector)
//  - indices in {0, ..., n-1} (compare to std::map)
//  - O(1) lookup
//  - O(n) removal (not implemented)
//
// It is essentially for a vector of objects which are expensive to copy.
template<class T>
class SmartVector {
  public:
    typedef typename std::list<T>::iterator iterator;
    typedef typename std::list<T>::const_iterator const_iterator;

    SmartVector();
    SmartVector(size_t n, const T& value);
    SmartVector(const SmartVector<T>& other);
    SmartVector& operator=(const SmartVector<T>& other);

    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;

    size_t size() const;
    bool empty() const;

    T& operator[](size_t n);
    const T& operator[](size_t n) const;
    T& front();
    const T& front() const;
    T& back();
    const T& back() const;

    void assign(size_t n, const T& value);
    void push_back(const T& x);
    void pop_back(const T& x);
    void swap(SmartVector<T>& other);
    void swap(std::list<T>& other);
    void clear();

  private:
    typedef std::list<T> List;
    typedef typename List::iterator ListEntry;

    std::list<T> list_;
    std::vector<ListEntry> vector_;

    void resetVector();
};

#include "smart_vector.inl"

#endif
