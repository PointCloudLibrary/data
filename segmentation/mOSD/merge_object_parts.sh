#!/bin/bash

merge()
{
  echo "Merging object parts in $1..."
  pcl_convert_pcd_ascii_binary $1 $1 0 2> /dev/null;
  sed -i "s/^[1-9] /1 /" $1
  sed -i "s/^\([0-9]\+\)[0-9]/\1/" $1
  pcl_convert_pcd_ascii_binary $1 $1 2 2> /dev/null;
}

export -f merge

if [[ $# -lt 2 ]]; then
  echo "Usage: ${0##*/} <input-dir> <output-dir> [num-threads]"
  exit 1
fi

if ! hash pcl_convert_pcd_ascii_binary 2>/dev/null; then
  echo "This script requires the tool 'pcl_convert_pcd_ascii_binary' (distributed"
  echo "as a part of the Point Cloud Library) to be present in the path."
  exit 2
fi

if [[ ! -d $1 ]]; then
  echo "Input folder does not exist!"
  exit 3
fi

mkdir -p $2

re='^[0-9]+$'

if [[ $# -eq 3 && $3 =~ $re ]]; then
  threads=$3
else
  threads=1
fi

cp $1/*.pcd $2/
ls $2/*.pcd | xargs -P $threads -n 1 -i bash -c 'merge "$@"' _ {}
