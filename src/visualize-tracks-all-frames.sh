#!/bin/bash

if [ $# -ne 3 ]
then
  echo "usage: $0 image-format tracks-format movie-format"
  exit
fi

image_format=$1
tracks_format=$2
movie_format=$3
render_dir=/tmp/$$

(( i = 1 ))
while [ -e `printf $image_format $i` ]
do
  echo $i

  tracks=`printf $tracks_format $i`
  frame_pattern=$render_dir/%d.png
  movie=`printf $movie_format $i`

  # Clear/create directory.
  rm -rf $render_dir
  mkdir $render_dir

  # Generate frames.
  (( n = i - 1 ))
  ./visualize-tracks $image_format $tracks $n --nodisplay --save --output_format $frame_pattern

  # Convert to movie.
  ffmpeg -y -loglevel quiet -i $frame_pattern -sameq -vcodec libx264 $movie

  # Remove directory.
  rm -rf $render_dir

  (( i += 1 ))
done
