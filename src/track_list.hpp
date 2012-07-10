#ifndef TRACK_LIST_HPP_
#define TRACK_LIST_HPP_

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include "track.hpp"

// Describes a collection of tracks from a single video sequence.
typedef std::vector<Track> TrackList;

// Saves a list of tracks to a file.
bool saveTracks(const std::string& filename,
                const cv::Size& size,
                const TrackList& tracks);

// Loads a list of tracks from a file.
bool loadTracks(const std::string& filename, cv::Size& size, TrackList& tracks);

// Returns the first frame in any track.
int findFirstFrame(const TrackList& tracks);

#endif
