#!/bin/sh -e

if test "$1" != 'skip'; then
	mkdir -p build
	cd build
	cmake -D CMAKE_BUILD_TYPE=Debug ..
	make
fi

case "$1" in
'skip') ;;
''|0|1|2|3|4|5)
	../gen_config.py "$1" ;;
esac

# launch the simulation
exec argos3 -c ./config.xml
