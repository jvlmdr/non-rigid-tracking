#!/bin/bash

if [ $# -ne 4 ]
then
  echo "usage: $0 image-format intrinsics frame-format movie"
  exit
fi

image_format=$1
calib=$2
frame_format=$3

(( i = 1 ))
while [ -e `printf $image_format $i` ]
do
  echo $i

  image=`printf $image_format $i`
  frame=`printf $frame_format $i`

  ./undistort-image $image $calib -nodisplay -save -output_file=$frame

  (( i += 1 ))
done

ffmpeg -y -sameq -i $frame_format $movie
