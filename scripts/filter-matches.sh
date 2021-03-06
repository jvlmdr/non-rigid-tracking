#!/bin/bash

if [ $# -ne 7 ]
then
  echo "usage: $0 input-format output-format options view1 time1 view2 time2"
  exit
fi

input_format=$1
output_format=$2
options=$3
view1=$4
time1=$5
view2=$6
time2=$7

input=`printf $input_format $view1 $view2 $time1 $time2`
output=`printf $output_format $view1 $view2 $time1 $time2`

echo "($view1, $time1), ($view2, $time2)"
./filter-matches $input $output $options
