#!/bin/bash

if [ $# -ne 3 ]
then
  echo "usage: $0 image-format features-format threshold"
  exit
fi

image_format=$1
features_format=$2
threshold=$3

(( i = 1 ))
while [ -e `printf $image_format $i` ]
do
  image=`printf $image_format $i`
  features=`printf $features_format $i`

  echo $i
  ./find-keypoints-and-extract-descriptors $image $features \
    --contrast_threshold=$threshold --logtostderr=1
  echo ""

  (( i += 1 ))
done
