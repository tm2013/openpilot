#!/usr/bin/bash

# Build openpilot

dirname=${PWD##*/}          # to assign to a variable
dirname=${dirname:-/}       # to correct for the case where PWD=/
if [ $dirname = "opgm_scripts" ]
then
  cd ..
fi

scons -j$(nproc)