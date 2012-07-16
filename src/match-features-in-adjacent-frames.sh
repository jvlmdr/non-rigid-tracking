#!/bin/bash

if [ $# -ne 3 ]
then
  echo "usage: $0 descriptors-format matches-format threshold"
  exit
fi

descriptors_format=$1
matches_format=$2
threshold=$3

(( j = 1 ))
while [ -e `printf $descriptors_format $j` ]
do
  (( i = j - 1 ))
  if [ "$i" -gt 0 ]
  then
    echo $i

    descriptors1=`printf $descriptors_format $i`
    descriptors2=`printf $descriptors_format $j`
    matches=`printf $matches_format $i`

    ./match-features $descriptors1 $descriptors2 $matches --threshold=$threshold
  fi

  (( j += 1 ))
done
