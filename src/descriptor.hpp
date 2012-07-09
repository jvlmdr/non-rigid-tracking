#ifndef DESCRIPTOR_HPP_
#define DESCRIPTOR_HPP_

#include <string>
#include <vector>

typedef std::vector<double> Descriptor;

// Saves a list of descriptors to a file.
bool saveDescriptors(const std::string& filename,
                     const std::vector<Descriptor>& descriptors);

// Loads a list of descriptors from a file.
bool loadDescriptors(const std::string& filename,
                     std::vector<Descriptor>& descriptors);

#endif
