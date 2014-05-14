cmake_minimum_required(VERSION 2.8)
project(m3ens)


set(M3ENS_ROOT ${PROJECT_SOURCE_DIR})

set(CMAKE_MODULE_PATH "${M3ENS_ROOT}/../meka-cmake-modules/Modules/" ${CMAKE_MODULE_PATH})

set(M3ENS_INCLUDE_DIR ${M3ENS_ROOT}/include)

set(M3ENS_PROTO_DIR ${M3ENS_ROOT}/proto)

set(M3ENS_SRC_DIR ${M3ENS_ROOT}/src)

set(M3ENS_INSTALL_BIN ${M3ENS_ROOT}/bin)

set(M3ENS_BINARY_OUTPUT_DIRECTORY ${M3ENS_ROOT}/bin)

set(M3ENS_INSTALL_LIB ${M3ENS_ROOT}/lib)

set(M3ENS_LIBRARY_OUTPUT_DIRECTORY ${M3ENS_ROOT}/lib)

set(M3ENS_PYTHON_DIR ${M3ENS_ROOT}/python)

add_subdirectory(src/m3ens)
