#!/bin/bash

if [ $# -ne 2 ]
then
  echo "usage: $0 data-file output-dir"
  exit
fi

data=$1
output_dir=$2

# Plot number of features/matches against contrast threshold.
gnuplot << EOF
set terminal postscript eps enhanced color
set output '$output_dir/features-versus-threshold.eps'

set log x
set xlabel 'Contrast threshold'
set ylabel 'Number of features'

set style line 1 lt 1 lw 5 pt 4 lc 1
set style line 2 lt 1 lw 5 pt 6 lc 2
set style line 3 lt 1 lw 5 pt 8 lc 3

plot '$data' using 1:2 with linespoints ls 1 axes x1y2 title 'Total', \
     '$data' using 1:3 with linespoints ls 2 axes x1y2 title 'Exhaustive matches', \
     '$data' using 1:5 with linespoints ls 3 axes x1y2 title 'FLANN matches'
EOF

# Plot time taken to do matching against contrast threshold.
gnuplot << EOF
set terminal postscript eps enhanced color
set output '$output_dir/time-versus-threshold.eps'

set log x
set xlabel 'Contrast threshold'
set ylabel 'Matching time (sec)'

set style line 1 lt 1 lw 5 pt 4 lc 1
set style line 2 lt 1 lw 5 pt 6 lc 2
set style line 3 lt 1 lw 5 pt 8 lc 3

plot '$data' using 1:4 with linespoints ls 2 axes x1y2 title 'Exhaustive matches', \
     '$data' using 1:6 with linespoints ls 3 axes x1y2 title 'FLANN matches'
EOF

# Plot matching time versus number of features obtained.
gnuplot << EOF
set terminal postscript eps enhanced color
set output '$output_dir/time-versus-matches.eps'

set xlabel 'Number of matches'
set ylabel 'Matching time (sec)'

set style line 1 lt 1 lw 5 pt 4 lc 1
set style line 2 lt 1 lw 5 pt 6 lc 2
set style line 3 lt 1 lw 5 pt 8 lc 3

plot '$data' using 3:4 with linespoints ls 2 axes x1y2 title 'Exhaustive matches', \
     '$data' using 5:6 with linespoints ls 3 axes x1y2 title 'FLANN matches'
EOF
