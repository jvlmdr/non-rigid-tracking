#!/bin/bash

if [ $# -ne 4 ]
then
  echo "usage: $0 image-format tracks-format render-dir-format render-file-format"
  exit
fi

image_format=$1
tracks_format=$2
render_dir_format=$3
render_file_format=$4

(( i = 1 ))
while [ -e `printf $image_format $i` ]
do
  echo $i

  tracks=`printf $tracks_format $i`
  render_dir=`printf $render_dir_format $i`
  (( n = i - 1 ))

  rm -rf $render_dir
  mkdir -p $render_dir
  ./visualize-tracks $image_format $tracks $n --nodisplay --save --output_format "$render_dir/$render_file_format"

  (( i += 1 ))
done
