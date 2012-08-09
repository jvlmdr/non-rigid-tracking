#!/bin/bash

if [ $# -ne 3 ]
then
  echo "usage: $0 distorted-format intrinsics undistorted-format"
  exit
fi

distorted_format=$1
calib=$2
undistorted_format=$3

(( i = 1 ))
while [ -e `printf $distorted_format $i` ]
do
  echo $i

  distorted=`printf $distorted_format $i`
  undistorted=`printf $undistorted_format $i`

  ./undistort-points $distorted $undistorted $calib

  (( i += 1 ))
done
