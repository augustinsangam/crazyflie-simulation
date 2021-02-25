FROM ubuntu:20.04

ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update

RUN apt-get install -y \
	cmake \
	freeglut3-dev \
	gcc \
	g++ \
	git \
	make \
	ninja-build \
	qt5-default \
	qtwayland5

WORKDIR /vendor/MISTLab
RUN git clone -b inf3995 --depth=1 https://github.com/MISTLab/argos3.git
WORKDIR /vendor/MISTLab/argos3-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D ARGOS_BUILD_FOR=simulator \
	-D ARGOS_THREADSAFE_LOG=ON \
	../argos3/src
RUN ninja
RUN touch argos3.1.gz && \
	ninja install
RUN ldconfig
RUN argos3 --version

WORKDIR /vendor/dermesser
RUN git clone -b v2.5.0 --depth=1 https://github.com/dermesser/libsocket.git
WORKDIR /vendor/dermesser/libsocket-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	../libsocket
RUN ninja
RUN ninja install

WORKDIR /vendor/fmtlib
RUN git clone -b 7.1.3 --depth=1 https://github.com/fmtlib/fmt.git
WORKDIR /vendor/fmtlib/fmt-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D FMT_DOC=OFF \
	-D FMT_TEST=OFF \
	../fmt
RUN ninja
RUN ninja install

WORKDIR /vendor/gabime
RUN git clone -b v1.8.2 --depth=1 https://github.com/gabime/spdlog.git
WORKDIR /vendor/gabime/spdlog-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D SPDLOG_BUILD_EXAMPLE=OFF \
	-D SPDLOG_ENABLE_PCH=ON \
	-D SPDLOG_FMT_EXTERNAL=ON \
	../spdlog
RUN ninja
RUN ninja install

WORKDIR /vendor/taocpp
RUN git clone -b 1.0.0-beta.12 --depth=1 https://github.com/taocpp/json.git
WORKDIR /vendor/taocpp/json-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D TAOCPP_JSON_BUILD_EXAMPLES=OFF \
	-D TAOCPP_JSON_BUILD_TESTS=OFF \
	../json
RUN ninja
RUN ninja install

WORKDIR /build

CMD cmake /drone && make && argos3 -c /drone/config.xml
