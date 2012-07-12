#ifndef DESCRIPTOR_SEQUENCE_HPP_
#define DESCRIPTOR_SEQUENCE_HPP_

#include <vector>
#include <map>
#include <string>
#include "descriptor.hpp"

// Describes a list of descriptors indexed by time.
typedef std::map<int, Descriptor> DescriptorSequence;
typedef DescriptorSequence::value_type IndexedDescriptor;

// Describes a collection of time-indexed descriptor lists.
typedef std::vector<DescriptorSequence> DescriptorSequenceList;

bool saveDescriptorSequences(const std::string& filename,
                             const DescriptorSequenceList& sequences);

#endif
