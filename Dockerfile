FROM ubuntu:20.04

ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update

RUN apt-get install -y \
	cmake \
	freeglut3-dev \
	g++ \
	git \
	make \
	ninja-build \
	qt5-default \
	qtwayland5

RUN git clone -b inf3995 --single-branch https://github.com/MISTLab/argos3.git

WORKDIR /argos3-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D ARGOS_BUILD_FOR=simulator \
	-D ARGOS_THREADSAFE_LOG=ON \
	/argos3/src
RUN ninja
RUN touch argos3.1.gz && \
	ninja install
RUN ldconfig
RUN argos3 --version

RUN apt-get install -y \
	rapidjson-dev

WORKDIR /build

CMD cmake /drone && make && argos3 -c /drone/config.xml
