#!/bin/bash

if [ $# -ne 13 ]
then
  echo usage: $0 descriptors1-format descriptors2-format matches-dir-format matches-file-format contrast-thresholds distance-thresholds use-flann-values reciprocal-values points1-format points2-format fund-mat residuals-dir-format residuals-file-format
  echo ""
  echo e.g. descriptorsX-format: descriptors/contrast/%s/%07d.yaml
  echo e.g. matches-dir-format: matches/contrast/%s/distance/%s/use-flann/%s/reciprocal/%s
  echo e.g. matches-file-format: %07d
  echo e.g. contrast-thresholds: \"0.01 0.02 0.05 0.1\"
  echo e.g. distance-thresholds: \"0.95 0.9 0.8 0.5\"
  echo e.g. use-flann-values: \"true false\"
  echo e.g. reciprocal-values: \"true false\"
  echo e.g. pointsX-format: undistorted/contrast/%s/%07d.yaml
  echo e.g. residuals-dir-format: residuals/contrast/%s/distance/%s/use-flann/%s/reciprocal/%s
  echo e.g. residuals-file-format: %07d
  exit
fi

descriptors_format1=$1
descriptors_format2=$2
matches_dir_format=$3
matches_file_format=$4
contrast_thresholds=$5
distance_thresholds=$6
use_flann_values=$7
reciprocal_values=$8
points_format1=$9
points_format2=${10}
fund_mat=${11}
residuals_dir_format=${12}
residuals_file_format=${13}

for contrast_threshold in $contrast_thresholds
do
  for distance_threshold in $distance_thresholds
  do
    for use_flann in $use_flann_values
    do
      for reciprocal in $reciprocal_values
      do
        matches_dir=`printf $matches_dir_format $contrast_threshold $distance_threshold $use_flann $reciprocal`
        residuals_dir=`printf $residuals_dir_format $contrast_threshold $distance_threshold $use_flann $reciprocal`
        rm -rf $matches_dir $residuals_dir
        mkdir -p $matches_dir $residuals_dir

        matches_format=$matches_dir/$matches_file_format
        residuals_format=$residuals_dir/$residuals_file_format

        (( i = 1 ))
        while [ -e `printf $descriptors_format1 $contrast_threshold $i` ]
        do
          echo "($contrast_threshold, $distance_threshold, $use_flann, $reciprocal, $i)"

          descriptors1=`printf $descriptors_format1 $contrast_threshold $i`
          descriptors2=`printf $descriptors_format2 $contrast_threshold $i`
          matches=`printf $matches_format $i`

          command="./match-features $descriptors1 $descriptors2 $matches -second_best_ratio=$distance_threshold -use_flann=$use_flann -reciprocal=$reciprocal -logtostderr"
          #echo "$command"
          $command

          points1=`printf $points_format1 $contrast_threshold $i`
          points2=`printf $points_format2 $contrast_threshold $i`
          residuals=`printf $residuals_format $i`

          command="./evaluate-matches $matches $points1 $points2 $fund_mat $residuals -logtostderr"
          #echo "$command"
          $command

          (( i += 1 ))
        done
      done
    done
  done
done
