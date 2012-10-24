#include <sstream>
#include <cstdlib>
#include <numeric>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/scoped_ptr.hpp>
#include <linear.h>

#include "classifier.hpp"
#include "descriptor.hpp"
#include "descriptor_reader.hpp"
#include "iterator_reader.hpp"
#include "iterator_writer.hpp"
#include "classifier_writer.hpp"

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Trains a classifier for every feature in the image." << std::endl;
  usage << std::endl;
  usage << argv[0] << " descriptors classifiers" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

typedef std::vector<feature_node> Example;

void descriptorToExample(const Descriptor& descriptor, Example& example) {
  example.clear();

  // Feature indices start at 1 not 0.
  int index = 1;

  // Copy non-zero elements.
  const Descriptor::Data& data = descriptor.data;
  Descriptor::Data::const_iterator x;
  for (x = data.begin(); x != data.end(); ++x) {
    if (*x != 0) {
      feature_node node;
      node.index = index;
      node.value = *x;

      example.push_back(node);
    }

    index += 1;
  }

  // Append bias term.
  feature_node one;
  one.index = index;
  one.value = 1;
  example.push_back(one);

  // Append terminator.
  feature_node end;
  end.index = -1;
  end.value = 0;
  example.push_back(end);
}

void descriptorsToExamples(const std::deque<Descriptor>& descriptors,
                           std::deque<Example>& examples) {
  examples.clear();

  std::deque<Descriptor>::const_iterator desc;
  for (desc = descriptors.begin(); desc != descriptors.end(); ++desc) {
    Example example;
    descriptorToExample(*desc, example);

    examples.push_back(Example());
    examples.back().swap(example);
  }
}

void nop(const char*) {}

double negate(double x) {
  return -x;
}

// n -- Dimensionality of vectors (not including the bias term).
void trainClassifier(const std::vector<const feature_node*>& examples,
                     const std::vector<double>& labels,
                     double C1,
                     double C2,
                     int n,
                     Classifier& classifier) {
  // Set up problem.
  problem prob;
  prob.l = examples.size();
  prob.n = n + 1;
  prob.y = const_cast<double*>(&labels.front());
  prob.x = const_cast<feature_node**>(&examples.front());
  prob.bias = 1;

  int weight_label[2] = { 1, 2 };
  double weight[2] = { C1, C2 };

  parameter param;
  param.solver_type = L2R_LR;
  param.eps = 1e-6;
  param.C = 1;
  param.nr_weight = 2;
  param.weight_label = weight_label;
  param.weight = weight;

  CHECK(check_parameter(&prob, &param) == NULL) <<
      "Invalid SVM problem definition";

  model* mod = train(&prob, &param);

  classifier.w.clear();
  // Copy first n elements into weight vector.
  std::copy(&mod->w[0], &mod->w[n], std::back_inserter(classifier.w));
  // Set bias to last element.
  classifier.b = mod->w[n];

  if (labels[0] == 1) {
    // First example was not in positive class. Negate w and b.
    // Not sure why this is necessary.
    std::transform(classifier.w.begin(), classifier.w.end(),
        classifier.w.begin(), negate);
    classifier.b = -classifier.b;
  }

  free_and_destroy_model(&mod);
}

void trainClassifiers(const std::deque<Descriptor>& descriptors,
                      std::deque<Classifier>& classifiers,
                      double C_pos,
                      double C_neg) {
  // Convert descriptors to liblinear examples.
  std::deque<Example> examples;
  descriptorsToExamples(descriptors, examples);

  // Obtain a list of pointers to the examples' data.
  std::vector<const feature_node*> data;
  std::deque<Example>::const_iterator example;
  for (example = examples.begin(); example != examples.end(); ++example) {
    data.push_back(&example->front());
  }

  size_t num_examples = descriptors.size();
  CHECK(!descriptors.empty());
  size_t num_dims = descriptors.front().data.size();
  std::vector<double> labels(num_examples, 1);
  classifiers.clear();

  for (size_t i = 0; i < num_examples; i += 1) {
    labels[i] = 2;

    Classifier classifier;
    trainClassifier(data, labels, C_neg, C_pos, num_dims, classifier);

    classifiers.push_back(Classifier());
    classifiers.back().swap(classifier);

    labels[i] = 1;
  }
}

void classify(const Classifier& classifier,
              const std::deque<Descriptor>& descriptors,
              std::vector<bool>& labels) {
  labels.clear();

  std::deque<Descriptor>::const_iterator descriptor;
  for (descriptor = descriptors.begin();
       descriptor != descriptors.end();
       ++descriptor) {
    bool label = classifier.classify(*descriptor);
    labels.push_back(label);
  }
}

void classify(const std::deque<Classifier>& classifiers,
              const std::deque<Descriptor>& descriptors,
              std::deque<std::vector<bool> >& labels) {
  labels.clear();

  std::deque<Classifier>::const_iterator classifier;
  for (classifier = classifiers.begin();
       classifier != classifiers.end();
       ++classifier) {
    // Classify each descriptor using this classifier.
    std::vector<bool> result;
    classify(*classifier, descriptors, result);

    // Swap the result into the list.
    labels.push_back(std::vector<bool>());
    labels.back().swap(result);
  }
}

struct ClassifierResult {
  int tp;
  int fp;
  int tn;
  int fn;

  ClassifierResult();

  double precision() const;
  double recall() const;
  double f1() const;

  ClassifierResult& operator+=(const ClassifierResult& rhs);
  ClassifierResult operator+(const ClassifierResult& rhs) const;
};

ClassifierResult::ClassifierResult() : tp(0), fp(0), tn(0), fn(0) {}

double ClassifierResult::precision() const {
  return double(tp) / (tp + fp);
}

double ClassifierResult::recall() const {
  return double(tp) / (tp + fn);
}

double ClassifierResult::f1() const {
  double p = precision();
  double r = recall();

  return (p * r) / (p + r);
}

ClassifierResult& ClassifierResult::operator+=(const ClassifierResult& rhs) {
  tp += rhs.tp;
  fp += rhs.fp;
  tn += rhs.tn;
  fn += rhs.fn;

  return *this;
}

ClassifierResult ClassifierResult::operator+(
    const ClassifierResult& rhs) const {
  ClassifierResult sum(*this);
  sum += rhs;
  return sum;
}

ClassifierResult evaluateClassifier(const Classifier& classifier,
                                    const std::deque<Descriptor>& descriptors,
                                    int i) {
  ClassifierResult result;

  std::deque<Descriptor>::const_iterator descriptor;
  int j = 0;

  for (descriptor = descriptors.begin();
       descriptor != descriptors.end();
       ++descriptor) {
    bool y = classifier.classify(*descriptor);

    if (y) {
      // Positive.
      if (j == i) {
        // Correct.
        result.tp += 1;
      } else {
        result.fp += 1; 
      }
    } else {
      // Negative.
      if (j != i) {
        // Correct.
        result.tn += 1;
      } else {
        result.fn += 1; 
      }
    }

    j += 1;
  }

  return result;
}

void evaluateClassifiers(const std::deque<Classifier>& classifiers,
                         const std::deque<Descriptor>& descriptors,
                         std::vector<ClassifierResult>& results) {
  results.clear();

  CHECK(classifiers.size() == descriptors.size());

  std::deque<Classifier>::const_iterator classifier;
  int i = 0;

  for (classifier = classifiers.begin();
       classifier != classifiers.end();
       ++classifier) {
    ClassifierResult result = evaluateClassifier(*classifier, descriptors, i);
    DLOG(INFO) << "precision " << (result.precision() * 100) << "%, recall " <<
        (result.recall() * 100) << "%";

    results.push_back(result);

    i += 1;
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string descriptors_file = argv[1];
  std::string classifiers_file = argv[2];

  // Load descriptors.
  DescriptorReader descriptor_reader;
  std::deque<Descriptor> descriptors;
  bool ok = loadList(descriptors_file, descriptors, descriptor_reader);
  CHECK(ok) << "Could not load descriptors from file";
  LOG(INFO) << "Loaded " << descriptors.size() << " descriptors";

  // Silence output of liblinear.
  set_print_string_function(nop);

  // Convert each descriptor to a training instance.
  std::deque<Classifier> classifiers;
  trainClassifiers(descriptors, classifiers, 0.5, 0.01);

  std::vector<ClassifierResult> results;
  evaluateClassifiers(classifiers, descriptors, results);

  // Add all results.
  ClassifierResult cumulative = std::accumulate(results.begin(), results.end(),
      ClassifierResult());
  LOG(INFO) << "Overall precision " << (cumulative.precision() * 100) <<
      "%, recall " << (cumulative.recall() * 100) << "%";

  ClassifierWriter classifier_writer;
  ok = saveList(classifiers_file, classifiers, classifier_writer);
  CHECK(ok) << "Could not save classifiers to file";

  return 0;
}
