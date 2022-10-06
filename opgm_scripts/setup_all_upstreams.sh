#!/usr/bin/bash

# Add the comma upstreams for the repo and subs

dirname=${PWD##*/}          # to assign to a variable
dirname=${dirname:-/}        # to correct for the case where PWD=/
echo $dirname
if [ $dirname = "opgm_scripts" ]
then
  echo "cd .."
  cd ..
fi

echo "Adding to openpilot"
git remote add upstream https://github.com/commaai/openpilot.git

cd cereal
echo "Adding to cereal"
git remote add upstream https://github.com/commaai/cereal.git
cd ..
cd opendbc
echo "Adding to opendbc"
git remote add upstream https://github.com/commaai/opendbc.git
cd ..
cd panda
echo "Adding to panda"
git remote add upstream https://github.com/commaai/panda.git
cd ..
