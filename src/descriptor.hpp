#ifndef DESCRIPTOR_HPP_
#define DESCRIPTOR_HPP_

#include <string>
#include <vector>
#include <boost/array.hpp>

typedef boost::array<double, 128> Descriptor;

// Saves a list of descriptors to a file.
bool saveDescriptors(const std::string& filename,
                     const std::vector<Descriptor>& descriptors);

#endif
