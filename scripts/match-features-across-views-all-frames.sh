#!/bin/bash

if [ $# -ne 3 ]
then
  echo "usage: $0 descriptors1-format descriptors2-format matches-format"
  exit
fi

descriptors_format1=$1
descriptors_format2=$2
matches_format=$3

(( i = 1 ))
while [ -e `printf $descriptors_format1 $i` ]
do
  echo $i

  descriptors1=`printf $descriptors_format1 $i`
  descriptors2=`printf $descriptors_format2 $i`
  matches=`printf $matches_format $i`

  ./match-features $descriptors1 $descriptors2 $matches -logtostderr

  (( i += 1 ))
done
