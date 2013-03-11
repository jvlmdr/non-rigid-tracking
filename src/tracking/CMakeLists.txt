PROTOBUF_GENERATE_CPP(
  TRACK_LIST_PROTO_SRC
  TRACK_LIST_PROTO_HDR
  track-list.proto)

add_library(tracking
  track-list.cpp
  warp.cpp
  flow.cpp
  translation-warp.cpp
  translation-warper.cpp
  similarity-warp.cpp
  similarity-warper.cpp
  ${TRACK_LIST_PROTO_SRC})

add_executable(detect-and-track detect-and-track.cpp)
target_link_libraries(detect-and-track
  tracking
  util
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARIES}
  ${CERES_LIBRARIES})
