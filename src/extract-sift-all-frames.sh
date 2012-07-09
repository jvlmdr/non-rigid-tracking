#!/bin/bash

if [ $# -ne 3 ]
then
  echo "usage: $0 image-format keypoints-format descriptors-format"
  echo ""
  echo "Example"
  echo "$0 input/%03d.png output/keypoints/%03d.yaml output/descriptors/%03d.yaml"
  exit
fi

image_format=$1
keypoints_format=$2
descriptors_format=$3

(( i = 1 ))
while [ -e `printf $image_format $i` ]
do
  image=`printf $image_format $i`
  keypoints=`printf $keypoints_format $i`
  descriptors=`printf $descriptors_format $i`

  echo $i
  ./extract-sift $image $keypoints $descriptors

  (( i += 1 ))
done
