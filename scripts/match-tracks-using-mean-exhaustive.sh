#!/bin/bash

if [ $# -ne 5 ]
then
  echo "usage: $0 descriptors-format views num-frames matches-format num-parallel"
  echo ""
  exit
fi

descriptors_format=$1
views=$2
num_frames=$3
matches_format=$4
num_parallel=$5

../scripts/generate-exhaustive-pairs.sh "$views" $num_frames | \
  xargs -n 4 -P $num_parallel \
  ../scripts/match-tracks-using-mean-helper.sh $descriptors_format $matches_format
