template<class T>
SmartVector<T>::SmartVector() : list_(), vector_() {}

template<class T>
SmartVector<T>::SmartVector(size_t n, const T& value)
    : list_(n, value), vector_() {
  resetVector();
}

template<class T>
SmartVector<T>::SmartVector(const SmartVector<T>& other)
    : list_(other.list_), vector_(other.vector_) {}

template<class T>
SmartVector<T>& SmartVector<T>::operator=(const SmartVector<T>& other) {
  list_ = other.list_;
  resetVector();

  return *this;
}

template<class T>
typename SmartVector<T>::iterator SmartVector<T>::begin() {
  return list_.begin();
}

template<class T>
typename SmartVector<T>::iterator SmartVector<T>::end() {
  return list_.end();
}

template<class T>
typename SmartVector<T>::const_iterator SmartVector<T>::begin() const {
  return list_.begin();
}

template<class T>
typename SmartVector<T>::const_iterator SmartVector<T>::end() const {
  return list_.end();
}

template<class T>
size_t SmartVector<T>::size() const {
  return list_.size();
}

template<class T>
bool SmartVector<T>::empty() const {
  return list_.empty();
}

template<class T>
const T& SmartVector<T>::operator[](size_t n) const {
  return *vector_[n];
}

template<class T>
T& SmartVector<T>::operator[](size_t n) {
  return *vector_[n];
}

template<class T>
const T& SmartVector<T>::front() const {
  return *vector_.front();
}

template<class T>
T& SmartVector<T>::front() {
  return *vector_.front();
}

template<class T>
const T& SmartVector<T>::back() const {
  return *vector_.back();
}

template<class T>
T& SmartVector<T>::back() {
  return *vector_.back();
}

template<class T>
void SmartVector<T>::assign(size_t n, const T& value) {
  list_.assign(n, value);
  resetVector();
}

template<class T>
void SmartVector<T>::push_back(const T& x) {
  list_.push_back(x);
  // Get iterator to last element.
  ListEntry last = list_.end();
  --last;
  // Add to vector.
  vector_.push_back(last);
}

template<class T>
void SmartVector<T>::pop_back(const T& x) {
  list_.pop_back();
  vector_.pop_back();
}

template<class T>
void SmartVector<T>::swap(SmartVector<T>& other) {
  list_.swap(other.list_);
  vector_.swap(other.vector_);
}

template<class T>
void SmartVector<T>::clear() {
  list_.clear();
  vector_.clear();
}

template<class T>
void SmartVector<T>::resetVector() {
  vector_.clear();

  for (ListEntry e = list_.begin(); e != list_.end(); ++e) {
    vector_.push_back(e);
  }
}
