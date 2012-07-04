CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(nrtrack)

# OpenCV
FIND_PACKAGE(OpenCV COMPONENTS core highgui imgproc)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# KLT
INCLUDE_DIRECTORIES("../klt")
LINK_DIRECTORIES("../klt")

ADD_EXECUTABLE(track-features track_features.cpp
  read_image.cpp
  tracker.cpp
  klt_tracker.cpp)
TARGET_LINK_LIBRARIES(track-features ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(track-features klt)

ADD_EXECUTABLE(extract-sift extract_sift.cpp
  read_image.cpp)
TARGET_LINK_LIBRARIES(extract-sift ${OpenCV_LIBS})
