#!/bin/bash

if [ $# -ne 8 ]
then
  echo "usage: $0 classifiers-format descriptors-format matches-format options view1 time1 view2 time2"
  exit
fi

classifiers_format=$1
descriptors_format=$2
matches_format=$3
options=$4
view1=$5
time1=$6
view2=$7
time2=$8

classifiers=`printf $classifiers_format $view1 $time1`
descriptors=`printf $descriptors_format $view2 $time2`
matches=`printf $matches_format $view1 $view2 $time1 $time2`

echo "($view1, $time1), ($view2, $time2)"
./match-features-using-classifiers $classifiers $descriptors $matches $options
