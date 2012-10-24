#ifndef CLASSIFIER_WRITER_HPP_
#define CLASSIFIER_WRITER_HPP_

#include "classifier.hpp"
#include "writer.hpp"

class ClassifierWriter : public Writer<Classifier> {
  public:
    ~ClassifierWriter();
    void write(cv::FileStorage& file, const Classifier& classifier);
};

#endif
