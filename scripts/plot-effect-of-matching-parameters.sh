#!/bin/bash

if [ $# -ne 5 ]
then
  echo "usage: $0 contrast-data distance-data contrast-output distance-output roc-output"
  exit
fi

contrast_data=$1
distance_data=$2
contrast_output=$3
distance_output=$4
roc_output=$5

gnuplot << EOF
set terminal postscript eps color enhanced
set output '$contrast_output'
set log xy
set xlabel 'Detector contrast threshold'
set ylabel 'Number of good matches'

plot '$contrast_data' u 1:4:(column(-2)) w lines lw 4 lc variable notitle, \
     ''               u 1:4:(column(-2)) w points pt 2 lc variable notitle
EOF

gnuplot << EOF
set terminal postscript eps color enhanced
set output '$distance_output'
set size 1,1
set log x
set xlabel 'First-vs-second match threshold (1 - x)'
set ylabel 'Fraction of matches which were bad'

plot '$distance_data' u (1-\$2):((\$3-\$4)/\$3):(column(-2)) w lines lw 4 lc variable notitle, \
     ''               u (1-\$2):((\$3-\$4)/\$3):(column(-2)) w points pt 2 lc variable notitle
EOF

gnuplot << EOF
set terminal postscript eps color enhanced
set output '$roc_output'
set size 1,1
set log y
set xlabel 'Fraction of matches which were bad (false discovery rate)'
set ylabel 'Number of good matches (true positive rate)'
set xrange [0:1]

plot '$contrast_data' u ((\$3-\$4)/\$3):4 w points lw 4 lc 1 pt 6 notitle, \
     ''               u ((\$3-\$4)/\$3):4 w lines lw 4 lc 2 title 'Varying detection threshold', \
     '$distance_data' u ((\$3-\$4)/\$3):4 w lines lw 4 lc 3 title 'Varying matching threshold'
EOF
