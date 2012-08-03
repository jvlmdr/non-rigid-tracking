#!/bin/bash

if [ $# -ne 4 ]
then
  echo "usage: $0 image-format features-dir features-file-format thresholds"
  exit
fi

image_format=$1
features_dir=$2
features_file_format=$3
thresholds=$4

for threshold in $thresholds
do
  threshold_dir=$features_dir/$threshold
  features_format=$threshold_dir/$features_file_format

  # Careful..
  rm -rf $threshold_dir
  mkdir -p $threshold_dir

  echo $threshold

  ../scripts/find-keypoints-and-extract-descriptors-all-frames.sh \
    $image_format $features_format $threshold
done
