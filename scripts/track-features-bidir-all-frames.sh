#!/bin/bash

if [ $# -ne 4 ]
then
  echo "usage: $0 image-format keypoints-format tracks-format max-frames"
  exit
fi

image_format=$1
keypoints_format=$2
tracks_format=$3
max_frames=$4

(( n = 1 ))
while [ -e `printf $image_format $n` ]
do
  (( i = n - 1 ))

  keypoints=`printf $keypoints_format $n`
  tracks=`printf $tracks_format $n`

  # Get some whitespace up in here.
  echo ""
  echo $n

  ./track-features-bidir $image_format $i $keypoints $tracks -max_frames $max_frames -nodisplay -logtostderr

  (( n += 1 ))
done
