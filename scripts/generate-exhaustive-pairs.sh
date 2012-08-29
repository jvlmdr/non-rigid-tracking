#!/bin/bash

if [ $# -ne 2 ]
then
  echo "usage: $0 views num-frames"
  echo ""
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
    for v2 in `seq $v1 $num_views`
    do
      view2=${views[((v2 - 1))]}

      if [ $v1 -eq $v2 ]
      then
        # Same view. Match next frame to end.
        (( a = t1 + 1 ))
      else
        # Different view. Match against every other frame.
        (( a = 1 ))
      fi

      for t2 in `seq $a $num_frames`
      do
        echo $view1 $t1 $view2 $t2
      done
    done
  done
done
