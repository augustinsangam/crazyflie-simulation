#!/bin/sh -e

case "$1" in
'build')
	if >/dev/null 2>&1 docker container inspect simulation; then
		docker container stop simulation
		docker container rm simulation
	fi
	docker image build -t simulation .
	DANGLING_IMAGES=$(docker images -q --filter dangling=true)
	case "$DANGLING_IMAGES" in
	'') ;;
	*) docker image rm $DANGLING_IMAGES
	esac
	exit 0
	;;
esac

if ! >/dev/null 2>&1 docker container inspect simulation; then
	docker container create \
		-e 'QT_QPA_PLATFORM=wayland' \
		-e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}" \
		-e 'XDG_RUNTIME_DIR=/tmp' \
		-v "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/tmp/${WAYLAND_DISPLAY}" \
		-v "${PWD}:/simulation" \
		--add-host=host.docker.internal:host-gateway \
		--name=simulation \
		simulation
fi

exec docker container start -ai simulation
