/*
 *  Created by Matthias Grundmann on 6/30/10.
 *  Copyright 2010 Matthias Grundmann. All rights reserved.
 *
 */

// Segmentation Reader and Writer to be used with segmentation.pb files.

// The binary, streaming file format we use has the following form:
// Each file contains various Headers followed by protobuffers containing the actual data.
// Format:
// Multiple chunks of the form:
// CHUNK_HEADER {
//   type                                         : char[4] = "CHNK"
//   Header ID                                    : int32
//   Number of frames in chunk (N)                : int32
//   N FileOffsets for SEG_FRAMES                 : int64
//   N TimeStamps (pts)                           : int64
//   FileOffset of next chunk header              : int64
// }
//
// followed by N frames
// SEG_FRAME {
//   type                                         : char[4] = "SEGD"
//   Size of protobuffer in bytes (sz)            : int32
//   Protobuffer serialized to binary format      : char[sz]
// }
//
// with terminating header at the end
// TERM_HEADER {
//   type                                         : char[4] = "TERM"
//   number of chunks                             : int32
// }

#ifndef VIDEOSEG_IO_HPP_
#define VIDEOSEG_IO_HPP_

#include "videoseg/using.hpp"
#include <fstream>

#ifdef __linux
#include <stdint.h>
#endif

#ifdef _WIN32
typedef __int64 int64_t;
typedef __int32 int32_t;
#endif

namespace videoseg {

class SegmentationDesc;

// Usage (user supplied variables in caps).
// SegmentationWriter writer(FILENAME);
// writer.OpenFile();
//
// for (int i = 0; i < NUM_FRAMES; ++i) {
//   writer.AddSegmentationToChunk(SEGMENTATION_FRAME[i], PTS[i]);
//   // When reasonable boundary is reached, write buffered chunk to file, e.g.
//   if (i + 1 % 10 == 0) {
//     writer.WriteChunk();
//   }
// }
//
// writer.WriteTermHeaderAndClose();

class SegmentationWriter {
public:
  SegmentationWriter(const string& filename)
      : filename_(filename), num_chunks_(0), curr_offset_(0) {}

  // Returns false if file could not be opened.
  bool OpenFile();

  // Buffers segmentation in chunk_buffer_.
  void AddSegmentationToChunk(const SegmentationDesc& desc, int64_t pts = 0);

  // Same as above if data was already serialized.
  void AddSegmentationDataToChunk(const char* data, int size, int64_t pts = 0);

  // Call to write whole chunk to file.
  void WriteChunk();

  // Finish file.
  void WriteTermHeaderAndClose();

  // Reuse writer for another file.
  void FlushAndReopen(const string& filename);
private:
  string filename_;
  std::ofstream ofs_;

  int num_chunks_;
  vector<string> chunk_buffer_;

  int64_t curr_offset_;
  vector<int64_t> file_offsets_;
  vector<int64_t> time_stamps_;
};

// Usage (user supplied variables in caps):
// SegmentationReader reader(FILE_NAME);
// reader.OpenFileAndReadHeaders();
// while (reader.RemainingFrames()) {
//   int frame_sz = reader.ReadNextFrameSize();
//   vector<unsigned char> buffer(frame_sz);
//   reader.ReadNextFrame(&buffer[0]);
//   
//   // Get segmentation protobuffer.
//   SegmentationDesc segmentation;
//   segmentation.ParseFromArray(&buffer[0], buffer.size());
//
//   // Process segmentation...
// }

class SegmentationReader {
public:
  SegmentationReader(const string& filename) :
      curr_frame_(0), filename_(filename) {}
  bool OpenFileAndReadHeaders();

  // Reads and parses first frame, returns resolution, seeks back to current playhead.
  void SegmentationResolution(int* width, int* height);

  // For each frame, first call ReadFrameSize
  // and subsequently ReadFrame.
  int ReadNextFrameSize();
  void ReadNextFrame(unsigned char* data);

  const vector<int64_t>& TimeStamps() { return time_stamps_; }
  void SeekToFrame(int frame);

  int NumFrames() const { return file_offsets_.size(); }
  int RemainingFrames() const { return NumFrames() - curr_frame_; }

  void CloseFile() { ifs_.close(); }
private:
  vector<int64_t> file_offsets_;
  vector<int64_t> time_stamps_;

  int frame_sz_;
  int curr_frame_;

  string filename_;
  std::ifstream ifs_;
};

}

#endif
