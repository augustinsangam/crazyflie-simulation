#!/bin/sh -e

case "$1" in
'build')
	if >/dev/null 2>&1 docker container inspect drone; then
		docker container stop drone
		docker container rm drone
	fi
	docker image build -q -t drone .
	DANGLING_IMAGES=$(docker images -q --filter dangling=true)
	case "$DANGLING_IMAGES" in
	'') ;;
	*) docker image rm $DANGLING_IMAGES
	esac
	exit 0
	;;
esac

if ! >/dev/null 2>&1 docker container inspect drone; then
	docker container create \
		-e 'QT_QPA_PLATFORM=wayland' \
		-e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}" \
		-e 'XDG_RUNTIME_DIR=/tmp' \
		-v "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/tmp/${WAYLAND_DISPLAY}" \
		-v "${PWD}:/drone" \
		--name=drone \
		drone
fi

exec docker container start -ai drone
