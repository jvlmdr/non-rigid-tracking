#!/bin/bash

if [ $# -ne 7 ]
then
  echo "usage: $0 descriptors-format matches-format options view1 time1 view2 time2"
  exit
fi

descriptors_format=$1
matches_format=$2
options=$3
view1=$4
time1=$5
view2=$6
time2=$7

descriptors1=`printf $descriptors_format $view1 $time1`
descriptors2=`printf $descriptors_format $view2 $time2`
matches=`printf $matches_format $view1 $view2 $time1 $time2`

echo "($view1, $time1), ($view2, $time2)"
./match-features $descriptors1 $descriptors2 $matches $options
