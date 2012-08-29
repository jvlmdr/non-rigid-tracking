#!/bin/bash

if [ $# -ne 4 ]
then
  echo "usage: $0 descriptors-format views num-frames matches-format"
  echo ""
  exit
fi

descriptors_format=$1
views=$2
num_frames=$3
matches_format=$4

../scripts/generate-exhaustive-pairs.sh "$views" $num_frames | \
  xargs -n 4 -P 4 ../scripts/match-tracks-using-min-helper.sh $descriptors_format $matches_format
