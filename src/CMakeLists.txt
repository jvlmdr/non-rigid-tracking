CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(nrtrack)

# OpenCV
FIND_PACKAGE(OpenCV COMPONENTS core highgui imgproc features2d nonfree)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# KLT
INCLUDE_DIRECTORIES("../klt")
LINK_DIRECTORIES("../klt")

# TBB for Shohei's SIFT
#ADD_DEFINITIONS(-DENABLE_TBB)
#INCLUDE_DIRECTORIES("/opt/tbb40_20120613oss/include")
#LINK_DIRECTORIES("/opt/tbb40_20120613oss/lib")

# Shohei's SIFT
#INCLUDE_DIRECTORIES("../sift-shohei")

ADD_EXECUTABLE(track-features track_features.cpp
  read_image.cpp
  tracker.cpp
  klt_tracker.cpp)
TARGET_LINK_LIBRARIES(track-features ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(track-features klt)

ADD_EXECUTABLE(find-keypoints find_keypoints.cpp
  read_image.cpp)
TARGET_LINK_LIBRARIES(find-keypoints ${OpenCV_LIBS})
#TARGET_LINK_LIBRARIES(find-keypoints tbb)

ADD_EXECUTABLE(extract-sift extract_sift.cpp
  read_image.cpp)
TARGET_LINK_LIBRARIES(extract-sift ${OpenCV_LIBS})
