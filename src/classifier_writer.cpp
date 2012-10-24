#include "classifier_writer.hpp"
#include <boost/bind.hpp>

namespace {

template<typename T>
bool writeToFile(cv::FileStorage& file, const T& x) {
  file << x;
  return true;
}

}

ClassifierWriter::~ClassifierWriter() {}

void ClassifierWriter::write(cv::FileStorage& file,
                             const Classifier& classifier) {
  file << "w" << "{";
  file << "list";
  file << "[:";
  std::for_each(classifier.w.begin(), classifier.w.end(),
      boost::bind(writeToFile<double>, boost::ref(file), _1));
  file << "]";
  file << "}";

  file << "b" << classifier.b;
}
