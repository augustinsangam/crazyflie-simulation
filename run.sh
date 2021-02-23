#!/bin/sh -e

docker build -q -t drone .
DANGLING_IMAGES=$(docker images -q --filter dangling=true)
case "$DANGLING_IMAGES" in
'') ;;
*) docker image rm $DANGLING_IMAGES
esac
exec docker run --rm \
	-e 'QT_QPA_PLATFORM=wayland' \
	-e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}" \
	-e 'XDG_RUNTIME_DIR=/tmp' \
	-v "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/tmp/${WAYLAND_DISPLAY}" \
	-v "${PWD}:/drone" \
	drone
