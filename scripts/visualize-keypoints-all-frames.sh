#!/bin/bash

if [ $# -ne 4 ]
then
  echo "usage: $0 image-format keypoints-format frame-format movie"
  exit
fi

image_format=$1
keypoints_format=$2
frame_format=$3
movie=$4

(( i = 1 ))
while [ -e `printf $image_format $i` ]
do
  echo $i

  image=`printf $image_format $i`
  keypoints=`printf $keypoints_format $i`
  frame=`printf $frame_format $i`

  ./visualize-keypoints $image $keypoints -nodisplay -save \
    -output_file=$frame -logtostderr=1

  (( i += 1 ))
done

ffmpeg -y -sameq -i $frame_format $movie
