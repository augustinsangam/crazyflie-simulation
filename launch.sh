#!/usr/bin/env bash

set -e

# change arena
ARENA_INDEX=$1
if [ $# -eq 0 ]
  then
	ARENA_INDEX=-1
fi

python arena_selector.py $ARENA_INDEX

sed -i "s/random_seed=\".*\" \/>/random_seed=\"$RANDOM\" \/>/g" config.xml

# build the project
mkdir -p build
cd build
cmake -D CMAKE_BUILD_TYPE=Debug ..
make

# launch the simulation
exec argos3 -c ../config.xml
