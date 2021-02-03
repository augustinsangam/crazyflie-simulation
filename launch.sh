#!/bin/sh -e

mkdir -p build
cd build
cmake -D CMAKE_BUILD_TYPE=Debug ..
make
argos3 -c ../config.xml
