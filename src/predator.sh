if [ $# -ne 4 ]
then
  echo "usage: $0 image-dir points tracks num-cores"
  exit
fi

image_dir=$1
points=$2
tracks=$3
num_cores=$4

matlab -nodisplay -singleCompThread -r "\
addpath('../src'); \
init_predator; \
try; \
  run_predator('$image_dir', '$points', '$tracks', $num_cores); \
catch e; \
  display(getReport(e)); \
end; \
exit;"
