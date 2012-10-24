#!/bin/bash

if [ $# -ne 8 ]
then
  echo "usage: $0 descriptors-format classifiers-format matches-format options view1 time1 view2 time2"
  exit
fi

descriptors_format=$1
classifiers_format=$2
matches_format=$3
options=$4
view1=$5
time1=$6
view2=$7
time2=$8

descriptors1=`printf $descriptors_format $view1 $time1`
descriptors2=`printf $descriptors_format $view2 $time2`
classifiers1=`printf $classifiers_format $view1 $time1`
classifiers2=`printf $classifiers_format $view2 $time2`
matches=`printf $matches_format $view1 $view2 $time1 $time2`

echo "($view1, $time1), ($view2, $time2)"
./match-features-using-classifiers $descriptors1 $descriptors2 $classifiers1 $classifiers2 $matches $options
