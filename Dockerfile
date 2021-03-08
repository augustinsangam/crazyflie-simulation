FROM ubuntu:20.10

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update
# toolchain
RUN apt-get install -y \
	binutils \
	libc-dev \
	libstdc++-10-dev \
	gcc-10 \
	g++-10
# tools
RUN apt-get install -y \
	cmake \
	git \
	make \
	ninja-build
# libraries
RUN apt-get install -y \
	freeglut3-dev \
	liblua5.4-dev \
	qtbase5-dev \
	qtwayland5

ENV CC=gcc-10 CXX=g++-10

WORKDIR /opt/src/MISTLab
RUN git clone -b inf3995 --depth=1 https://github.com/MISTLab/argos3.git
RUN apt-get install -y curl patch
RUN curl https://github.com/ilpincy/argos3/commit/d954c4b1797b830b3f7a703e22c19d2d447d2747.patch \
	| patch -d argos3 -p 1
WORKDIR /opt/src/MISTLab/argos3-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_INSTALL_PREFIX=/opt \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D ARGOS_BUILD_FOR=simulator \
	-D ARGOS_THREADSAFE_LOG=ON \
	../argos3/src
RUN ninja
RUN touch argos3.1.gz && \
	ninja install

WORKDIR /opt/src/dermesser
RUN git clone -b v2.5.0 --depth=1 https://github.com/dermesser/libsocket.git
WORKDIR /opt/src/dermesser/libsocket-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_INSTALL_PREFIX=/opt \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	../libsocket
RUN ninja
RUN ninja install

WORKDIR /opt/src/fmtlib
RUN git clone -b 7.1.3 --depth=1 https://github.com/fmtlib/fmt.git
WORKDIR /opt/src/fmtlib/fmt-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_INSTALL_PREFIX=/opt \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D FMT_DOC=OFF \
	-D FMT_TEST=OFF \
	../fmt
RUN ninja
RUN ninja install

WORKDIR /opt/src/gabime
RUN git clone -b v1.8.2 --depth=1 https://github.com/gabime/spdlog.git
WORKDIR /opt/src/gabime/spdlog-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_INSTALL_PREFIX=/opt \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D SPDLOG_BUILD_EXAMPLE=OFF \
	-D SPDLOG_ENABLE_PCH=ON \
	-D SPDLOG_FMT_EXTERNAL=ON \
	../spdlog
RUN ninja
RUN ninja install

WORKDIR /opt/src/taocpp
RUN git clone -b 1.0.0-beta.12 --depth=1 https://github.com/taocpp/json.git
WORKDIR /opt/src/taocpp/json-build
RUN cmake \
	-G Ninja \
	-D CMAKE_BUILD_TYPE=Release \
	-D CMAKE_INSTALL_PREFIX=/opt \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D TAOCPP_JSON_BUILD_EXAMPLES=OFF \
	-D TAOCPP_JSON_BUILD_TESTS=OFF \
	../json
RUN ninja
RUN ninja install

WORKDIR /build
ENV LD_LIBRARY_PATH=/opt/lib/argos3

CMD cmake /simulation && make && /opt/bin/argos3 -c /simulation/config.xml
