if [ $# -ne 3 ]
then
  echo "usage: $0 image-format keypoints-format tracks-format"
  echo ""
  echo "Example"
  echo "$0 input/my-video/%03d.png output/my-video/keypoints/%03d.yaml output/my-video/tracks/%03d.yaml"
  exit
fi

image_format=$1
keypoints_format=$2
tracks_format=$3

(( i = 1 ))
while [ -e `printf $image_format $i` ]
do
  keypoints=`printf $keypoints_format $i`
  tracks=`printf $tracks_format $i`

  echo $i
  (( n = i - 1 ))
  ./track-features-bidirectional $image_format $n $keypoints $tracks

  # Get some whitespace up in here.
  echo ""

  (( i += 1 ))
done
