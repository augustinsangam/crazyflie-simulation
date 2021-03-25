cmake_minimum_required(VERSION 3.16)
project(simulation VERSION 0.1.0 LANGUAGES CXX)

include(CheckCXXCompilerFlag)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(common INTERFACE)
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
	target_compile_options(common INTERFACE "-Wall" "-Wextra" "-Wpedantic")
	target_compile_options(common INTERFACE
		"-Wcast-qual"
		"-Wconversion"
		"-Wdouble-promotion"
		"-Wformat=2"
		"-Wmissing-declarations"
		"-Wshadow"
		"-Wstrict-overflow")
	target_compile_options(common INTERFACE
		"-Wno-missing-field-initializers")
endif()
check_cxx_compiler_flag("-fpic" MINIMAL_FPIC_SUPPORTED)
if(MINIMAL_FPIC_SUPPORTED)
	set(FPIC "-fpic")
else()
	set(FPIC "-fPIC")
endif()
check_cxx_compiler_flag("-fpie" MINIMAL_FPIE_SUPPORTED)
if (MINIMAL_FPIE_SUPPORTED)
	set(FPIE "-fpie")
else()
	set(FPIE "-fPIE")
endif()

find_package(spdlog REQUIRED)
find_package(taocpp-json REQUIRED)

set(EXPLORATION_DEP ON)
set(EXPLORATION_METHOD 1)
set(EXPLORATION_DRONE_INITIALISATION_DELAY 0)
add_subdirectory("./exploration")
target_compile_options(exploration PRIVATE "${FPIC}")

add_library(controller SHARED)
target_compile_options(controller PRIVATE "${FPIC}")
target_include_directories(controller PRIVATE "./include")
target_link_libraries(controller PRIVATE common)
target_link_libraries(controller PRIVATE spdlog::spdlog taocpp::json)
target_link_libraries(controller PRIVATE exploration)
target_sources(controller PRIVATE
	"./src/conn/Conn.cpp"
	"./src/Brain.cpp"
	"./src/Decoder.cpp"
	"./src/Proxy.cpp"
	"./src/RTStatus.cpp"
	"./src/main.cpp")
