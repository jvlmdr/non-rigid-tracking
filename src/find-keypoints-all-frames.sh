if [ $# -ne 2 ]
then
  echo "usage: $0 image-format keypoints-format"
  echo ""
  echo "Example"
  echo "$0 input/my-video/%03d.png output/my-video/keypoints-%03d.yaml"
  exit
fi

image_format=$1
keypoints_format=$2

(( i = 1 ))
while [ -e `printf $image_format $i` ]
do
  image=`printf $image_format $i`
  keypoints=`printf $keypoints_format $i`

  ./find-keypoints $image $keypoints
  echo $i

  (( i += 1 ))
done
