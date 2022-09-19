#!/usr/bin/bash

cd ~/openpilot
scons -j$(nproc)
