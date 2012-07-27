#!/bin/bash

if [ $# -ne 3 ]
then
  echo "usage: $0 image-format tracks-format descriptors-format"
  exit
fi

image_format=$1
tracks_format=$2
descriptors_format=$3

(( n = 1 ))
while [ -e `printf $image_format $n` ]
do
  (( i = n - 1 ))
  # Get some whitespace up in here.
  echo ""
  echo $n

  tracks=`printf $tracks_format $n`
  descriptors=`printf $descriptors_format $n`

  command="./extract-sift-tracks $tracks $image_format $descriptors"
  echo "$command"
  $command
  rc=$?

  if [ $rc -ne 0 ]
  then
    exit $rc
  fi

  (( n += 1 ))
done
