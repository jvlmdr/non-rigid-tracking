#!/bin/bash

if [ $# -ne 9 ]
then
  echo "usage: $0 descriptors1-format descriptors2-format matches-format threshold image1-format image2-format keypoints1-format keypoints2-format render-format"
  exit
fi

descriptors_format1=$1
descriptors_format2=$2
matches_format=$3
threshold=$4
image_format1=$5
image_format2=$6
keypoints_format1=$7
keypoints_format2=$8
render_format=$9

(( i = 1 ))
while [ -e `printf $image_format1 $i` ]
do
  echo $i

  descriptors1=`printf $descriptors_format1 $i`
  descriptors2=`printf $descriptors_format2 $i`
  matches=`printf $matches_format $i`

  ./match-features $descriptors1 $descriptors2 $matches --threshold=$threshold

  image1=`printf $image_format1 $i`
  image2=`printf $image_format2 $i`
  keypoints1=`printf $keypoints_format1 $i`
  keypoints2=`printf $keypoints_format2 $i`
  render=`printf $render_format $i`

  ./visualize-matches $matches $image1 $image2 $keypoints1 $keypoints2 --save --nodisplay --output_file=$render

  (( i += 1 ))
done
