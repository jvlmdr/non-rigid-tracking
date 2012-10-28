#!/bin/bash

if [ $# -ne 2 ]
then
  echo "usage: $0 views num-frames" >&2
  exit
fi

views=$1
num_frames=$2

# Replace views string with an array.
declare -a views=($views)
num_views=${#views[@]}

for v1 in `seq 1 $num_views`
do
  view1=${views[((v1 - 1))]}

  for t1 in `seq 1 $num_frames`
  do
    for v2 in `seq 1 $num_views`
    do
      view2=${views[((v2 - 1))]}

      for t2 in `seq 1 $num_frames`
      do
        if [ $v1 -ne $v2 -o $t1 -ne $t2 ]
        then
          # Print pairs.
          echo $view1 $t1 $view2 $t2
        fi
      done
    done
  done
done
