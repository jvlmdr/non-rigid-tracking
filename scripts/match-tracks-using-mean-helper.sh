#!/bin/bash

if [ $# -ne 6 ]
then
  echo "usage: $0 descriptors-format matches-format view1 time1 view2 time2"
  exit
fi

descriptors_format=$1
matches_format=$2
view1=$3
time1=$4
view2=$5
time2=$6

descriptors1=`printf $descriptors_format $view1 $time1`
descriptors2=`printf $descriptors_format $view2 $time2`
matches=`printf $matches_format $view1 $view2 $time1 $time2`

echo "($view1, $time1), ($view2, $time2)"
./match-tracks-using-mean $descriptors1 $descriptors2 $matches -logtostderr
