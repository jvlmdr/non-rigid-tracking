#!/bin/bash

if [ $# -ne 4 ]
then
  echo "usage: $0 num-frames views descriptors-format matches-format"
  echo ""
  exit
fi

num_frames=$1
views=$2
descriptors_format=$3
matches_format=$4

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
        echo "(($view1, $t1), ($view2, $t2))"

        descriptors1=`printf $descriptors_format $view1 $t1`
        descriptors2=`printf $descriptors_format $view2 $t2`
        matches=`printf $matches_format $view1 $view2 $t1 $t2`

        ./match-features $descriptors1 $descriptors2 $matches
      done
    done
  done
done
