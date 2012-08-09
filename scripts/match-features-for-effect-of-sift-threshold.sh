#!/bin/bash

if [ $# -ne 5 ]
then
  echo "usage: $0 features1-format features2-format matches-dir-format matches-file-format thresholds"
  exit
fi

features_format1=$1
features_format2=$2
matches_dir_format=$3
matches_file_format=$4
thresholds=$5

for threshold in $thresholds
do
  echo $threshold

  features1=`printf $features_format1 $threshold`
  features2=`printf $features_format2 $threshold`
  matches_dir=`printf $matches_dir_format $threshold`
  matches_format=$matches_dir/$matches_file_format

  # Careful..
  rm -rf $matches_dir
  mkdir -p $matches_dir

  ../scripts/match-features-across-views-all-frames.sh \
    $features1 $features2 $matches_format
done
