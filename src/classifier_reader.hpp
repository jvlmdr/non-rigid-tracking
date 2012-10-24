#ifndef CLASSIFIER_READER_HPP_
#define CLASSIFIER_READER_HPP_

#include "classifier.hpp"
#include "reader.hpp"

class ClassifierReader : public Reader<Classifier> {
  public:
    ~ClassifierReader();
    bool read(const cv::FileNode& node, Classifier& classifier);
};

#endif
