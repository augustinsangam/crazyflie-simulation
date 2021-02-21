#!/bin/sh -e

# build the project
mkdir -p build
cd build
cmake -D CMAKE_BUILD_TYPE=Debug ..
make

# change random seed for obstacle generation
# sed -i "s/random_seed=\".*\" \/>/random_seed=\"$RANDOM\" \/>/g" ../config.xml

# launch the simulation
argos3 -c ../config.xml
