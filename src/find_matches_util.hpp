#include <vector>
#include <opencv2/core/core.hpp>
#include "descriptor.hpp"

// Copies a list of descriptors into a matrix of descriptor rows.
void listToMatrix(const std::vector<Descriptor>& list, cv::Mat& matrix);
