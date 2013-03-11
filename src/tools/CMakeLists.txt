add_executable(select-foreground-tracks select-foreground-tracks.cpp)
target_link_libraries(select-foreground-tracks
  videoseg
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARIES})
