cmake_minimum_required(VERSION 3.12.4)
project(Project LANGUAGES CXX)

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
		"-Wformat=2"
		"-Wmissing-declarations"
		"-Wshadow"
		"-Wstrict-overflow")
	target_compile_options(common INTERFACE
		"-Wno-missing-field-initializers")
endif()
check_cxx_compiler_flag("-fpic" IS_MINIMAL_FPIC_SUPPORTED)
if(IS_MINIMAL_FPIC_SUPPORTED)
	set(FPIC "-fpic")
else()
	set(FPIC "-fPIC")
endif()
check_cxx_compiler_flag("-fpie" IS_MINIMAL_FPIE_SUPPORTED)
if (IS_MINIMAL_FPIE_SUPPORTED)
	set(FPIE "-fpie")
else()
	set(FPIE "-fPIE")
endif()

add_library(controller SHARED)
target_compile_options(controller PRIVATE "${FPIC}")
target_compile_definitions(controller PRIVATE RAPIDJSON_HAS_STDSTRING=1)
target_include_directories(controller PRIVATE include)
file(GLOB CONTROLLER_SRC src/*)
target_sources(controller PRIVATE ${CONTROLLER_SRC})
