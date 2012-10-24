#!/bin/bash

if [ $# -ne 7 ]
then
  echo "usage: $0 descriptors-format classifiers-format matches-format view1 time1 view2 time2"
  exit
fi

descriptors_format=$1
classifiers_format=$2
matches_format=$3
view1=$4
time1=$5
view2=$6
time2=$7

descriptors1=`printf $descriptors_format $view1 $time1`
descriptors2=`printf $descriptors_format $view2 $time2`
classifiers1=`printf $classifiers_format $view1 $time1`
classifiers2=`printf $classifiers_format $view2 $time2`
matches=`printf $matches_format $view1 $view2 $time1 $time2`

echo "($view1, $time1), ($view2, $time2)"
./match-features-using-classifiers $descriptors1 $descriptors2 $classifiers1 $classifiers2 $matches -logtostderr
