PROTOBUF_GENERATE_CPP(
  REGION_PROTO_SRC
  REGION_PROTO_HDR
  region.proto)

PROTOBUF_GENERATE_CPP(
  HIERARCHICAL_SEGMENTATION_PROTO_SRC
  HIERARCHICAL_SEGMENTATION_PROTO_HDR
  hierarchical-segmentation.proto)

PROTOBUF_GENERATE_CPP(
  SEGMENTATION_PROTO_SRC
  SEGMENTATION_PROTO_HDR
  segmentation.proto)

add_library(videoseg
  hierarchical-segmentation.cpp
  io.cpp
  tree.cpp
  segmentation.cpp
  region.cpp
  ${REGION_PROTO_SRC}
  ${HIERARCHICAL_SEGMENTATION_PROTO_SRC}
  ${SEGMENTATION_PROTO_SRC})

add_library(videoseg-draw draw-region.cpp ${REGION_PROTO_SRC})

add_executable(segment-foreground segment-foreground.cpp)
target_link_libraries(segment-foreground
  videoseg videoseg-draw util
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARIES})

add_executable(visualize-foreground visualize-foreground.cpp)
target_link_libraries(visualize-foreground
  videoseg videoseg-draw util
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARIES})
