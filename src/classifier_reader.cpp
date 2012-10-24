#include "classifier_reader.hpp"
#include "iterator_reader.hpp"
#include "default_reader.hpp"

ClassifierReader::~ClassifierReader() {}

bool ClassifierReader::read(const cv::FileNode& node, Classifier& classifier) {
  if (node.type() != cv::FileNode::MAP) {
    return false;
  }

  classifier.w.clear();
  InlineDefaultReader<double> reader;
  bool ok = readSequence(node["w"], reader, std::back_inserter(classifier.w));
  if (!ok) {
    return false;
  }

  ok = ::read<double>(node["b"], classifier.b);
  if (!ok) {
    return false;
  }

  return true;
}
