#!/usr/bin/bash

# Add the comma upstreams for the repo and subs
# NOTE: Hardcoded to ~/openpilot

cd ~/openpilot

git remote add upstream https://github.com/commaai/openpilot.git
cd cereal
git remote add upstream https://github.com/commaai/cereal.git
cd ..
cd opendbc
git remote add upstream https://github.com/commaai/opendbc.git
cd ..
cd panda
git remote add upstream https://github.com/commaai/panda.git
cd ..
