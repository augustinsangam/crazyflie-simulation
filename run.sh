#!/bin/sh

exec docker run \
	-e 'QT_QPA_PLATFORM=wayland' \
	-e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}" \
	-e 'XDG_RUNTIME_DIR=/tmp' \
	-v "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/tmp/${WAYLAND_DISPLAY}" \
	drone
