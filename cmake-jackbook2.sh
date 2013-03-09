export CMAKE_INCLUDE_PATH=\
/usr/local/include/eigen3/:\
~/src/ceres-solver/include/:\
~/src/liblinear-1.92/

export CMAKE_LIBRARY_PATH=\
~/src/ceres-solver/release/internal/ceres/:\
~/src/liblinear-1.92/

export GTEST_ROOT=~/src/gtest-1.6.0/release/

cmake ../src/ -DCMAKE_MODULE_PATH=~/cmake/Modules/ "$@"
