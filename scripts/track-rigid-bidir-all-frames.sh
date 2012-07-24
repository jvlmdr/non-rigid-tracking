#!/bin/bash

if [ $# -ne 6 ]
then
  echo "usage: $0 image-format keypoints-format tracks-format max-frames frame-dir-format video-format"
  echo ""
  echo "Example"
  echo "$0 input/my-video/%03d.png output/my-video/keypoints/%03d.yaml output/my-video/tracks/%03d.yaml"
  exit
fi

image_format=$1
keypoints_format=$2
tracks_format=$3
max_frames=$4
frame_dir_format=$5
video_format=$6

(( n = 1 ))
while [ -e `printf $image_format $n` ]
do
  (( i = n - 1 ))

  keypoints=`printf $keypoints_format $n`
  tracks=`printf $tracks_format $n`
  frame_dir=`printf $frame_dir_format $n`
  video=`printf $video_format $n`

  frame_format="$frame_dir/%d.png"

  # Get some whitespace up in here.
  echo ""
  echo "========================================"
  echo $n

  # Careful...
  rm -rf $frame_dir
  mkdir $frame_dir

  ./track-rigid-bidir $image_format $i $keypoints $tracks --max_frames $max_frames --nodisplay --save --save_format $frame_format 2> /dev/null

  ffmpeg -y -sameq -i $frame_format $video

  (( n += 1 ))
done
