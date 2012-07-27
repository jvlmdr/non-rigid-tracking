#!/bin/bash

if [ $# -ne 2 ]
then
  echo "usage: $0 descriptor-tracks-format plot-format"
  exit
fi

descriptors_format=$1
plot_format=$2

data=/tmp/$$.dat

(( n = 1 ))
while [ -e `printf $descriptors_format $n` ]
do
  echo $n

  descriptors=`printf $descriptors_format $n`
  plot=`printf $plot_format $n`

  ../scripts/visualize-pca-descriptors.sh $descriptors $plot

  (( n += 1 ))
done
