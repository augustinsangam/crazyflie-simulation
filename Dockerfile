FROM ubuntu:20.04

ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update

RUN apt-get install -y \
	cmake \
	freeglut3-dev \
	g++ \
	git \
	ninja-build \
	qt5-default

RUN git clone --single-branch --branch inf3995 https://github.com/MISTLab/argos3.git

WORKDIR /argos3-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D ARGOS_BUILD_FOR=simulator \
	-D ARGOS_THREADSAFE_LOG=ON \
	../argos3/src
RUN ninja
RUN touch argos3.1.gz && \
	ninja install
RUN ldconfig
RUN argos3 --version

RUN apt-get install rapidjson-dev uuid-dev
RUN mkdir /drone
COPY include /drone/include
COPY src /drone/src
COPY CMakeLists.txt /drone/CMakeLists.txt
COPY config.xml /drone/config.xml

WORKDIR /build
RUN cmake \
	-G Ninja \
	../drone
RUN ninja

CMD argos3 -c /drone/config.xml
