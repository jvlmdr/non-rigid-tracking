# Boost
find_package(Boost REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED
  COMPONENTS core highgui imgproc features2d nonfree video)
include_directories(${OpenCV_INCLUDE_DIRS})

# Protocol Buffers (required by Ceres)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

# Ceres
find_path(CERES_INCLUDE_DIRS NAMES ceres/ceres.h)
find_library(CERES_LIBRARIES NAMES ceres)
include_directories(${CERES_INCLUDE_DIRS})
# SuiteSparse (for Ceres)
find_library(CXSPARSE_LIBRARIES NAMES cxsparse)
find_library(CHOLMOD_LIBRARIES NAMES cholmod)
find_library(CCOLAMD_LIBRARIES NAMES ccolamd)
find_library(CAMD_LIBRARIES NAMES camd)
find_library(COLAMD_LIBRARIES NAMES colamd)
find_library(AMD_LIBRARIES NAMES amd)
set(SUITESPARSE_LIBRARIES
  ${CXSPARSE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${CCOLAMD_LIBRARIES}
  ${CAMD_LIBRARIES}
  ${COLAMD_LIBRARIES}
  ${AMD_LIBRARIES})
# LAPACK (for Ceres)
find_package(LAPACK REQUIRED)
# Eigen (for Ceres)
find_path(EIGEN_INCLUDE_DIRS NAMES Eigen/Core)
include_directories(${EIGEN_INCLUDE_DIRS})
set(CERES_LIBRARIES
  ${CERES_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${LAPACK_LIBRARIES}
  ${PROTOBUF_LIBRARIES})

# Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Google Log
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# GFlags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# liblinear
find_path(LIBLINEAR_INCLUDE_DIRS NAMES linear.h)
include_directories(${LIBLINEAR_INCLUDE_DIRS})
find_library(LIBLINEAR_LIBRARIES NAMES linear)
set(LIBLINEAR_LIBRARIES ${LIBLINEAR_LIBRARIES} ${BLAS_LIBRARIES})

# GNU Scientific Library (GSL)
find_library(GSL_LIBRARIES NAMES gsl)
