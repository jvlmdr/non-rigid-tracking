#!/bin/bash

if [ $# -ne 2 ]
then
  echo "usage: $0 descriptor-tracks-file plot-file"
  exit
fi

descriptors=$1
plot=$2

data=/tmp/$$.dat

./pca-descriptor $descriptors > $data

gnuplot << EOF
set terminal postscript eps color enhanced
set output "$2"
set size ratio -1

plot "$data" using 1:2:3 notitle with lines lc rgb variable
EOF
