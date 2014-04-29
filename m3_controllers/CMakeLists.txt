# States that CMake required version must be greater than 2.8
cmake_minimum_required(VERSION 2.8)

# Project name is not mandatory, but you should use it
project(m3_controllers)

###############################################################################
# General settings

# Some settings related to building/installing libraries and header files
# Set this to STATIC if you want to build static libraries instead.
set(SHARED_OR_STATIC "STATIC")
set(LIB_INSTALL_DIR /usr/local/lib/${PROJECT_NAME})
set(INCLUDE_INSTALL_DIR /usr/local/include/${PROJECT_NAME})
#set(BIN_INSTALL_DIR /usr/local/bin)
# Local install path
#set(LIB_INSTALL_DIR ${CMAKE_SOURCE_DIR}/lib)
#set(INCLUDE_INSTALL_DIR ${CMAKE_SOURCE_DIR}/include)
set(BIN_INSTALL_DIR ${CMAKE_SOURCE_DIR}/bin)
set(PYTHON_INSTALL_DIR ${CMAKE_SOURCE_DIR}/../python/m3ens) #TO FIX

# Some flags for CXX
set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
#set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

# Appends the cmake/modules path inside the MAKE_MODULE_PATH variable which stores the
# directories of additional CMake modules (ie. MacroOutOfSourceBuild.cmake):
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules" ${CMAKE_MODULE_PATH})

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)

###############################################################################
# Find RTAI
find_package(RTAI REQUIRED)
set(LIBRARIES ${LIBRARIES} ${RTAI_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${RTAI_INCLUDE_DIR})

#message(RTAI_LIBRARIES ${RTAI_LIBRARIES})
#message(RTAI_INCLUDE_DIR ${RTAI_INCLUDE_DIR})

###############################################################################
# Find M3
find_package(M3 COMPONENTS HARDWARE ROBOTS CHAINS TOOLBOX SHARED_MEM REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3_INCLUDE_DIR})

#message(M3_LIBRARIES ${M3_LIBRARIES})
#message(M3_INCLUDE_DIR ${M3_INCLUDE_DIR})

###############################################################################
# Find M3RT
find_package(M3rt REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3RT_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3RT_INCLUDE_DIR})

#message(M3RT_LIBRARIES ${M3RT_LIBRARIES})
#message(M3RT_INCLUDE_DIR ${M3RT_INCLUDE_DIR})

###############################################################################
# Find google protobuff
find_package(Protobuf REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})
set(PROTOBUF_IMPORT_DIRS ${M3RT_INCLUDE_DIR} ${M3_INCLUDE_DIR})
# Take the protofiles from the m3 system
#file(GLOB M3SysProtoFiles "/usr/local/include/m3rt/base/*.proto" 
#		          "/usr/local/include/m3/toolbox/*.proto"
#		          "/usr/local/include/m3/hardware/*.proto" 
#		          "/usr/local/include/m3/chain/*.proto")

###############################################################################
# Find Eigen
find_package(Eigen3 REQUIRED)
set(LIBRARIES ${LIBRARIES} ${EIGEN3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

###############################################################################
# Find boost packages
find_package(Boost 1.34 COMPONENTS filesystem system serialization REQUIRED)
set(LIBRARIES ${LIBRARIES} ${Boost_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

###############################################################################
# Find Catkin components
#find_package(catkin REQUIRED COMPONENTS kdl_kinematics)
#set(LIBRARIES ${LIBRARIES} ${catkin_LIBRARIES})
#set(INCLUDE_DIRS ${INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

###############################################################################
# Dl library (dynamic linking) NOTE Fixed by the new m3ens cmake build system, waiting for the migration
set(LIBRARIES ${LIBRARIES} ${CMAKE_DL_LIBS})

###############################################################################
# Include directories
include_directories(${INCLUDE_DIRS})

###############################################################################
# Things to compile are in the src/ directory
add_subdirectory(src)

###############################################################################
# Create the m3_controllers library, to be loaded in the m3_rt_factory
add_library(${PROJECT_NAME} SHARED src/factory_proxy.cpp)
#set_target_properties(m3_controllers PROPERTIES LINKER_LANGUAGE CXX)

###############################################################################
# Linking m3_controllers
target_link_libraries(${PROJECT_NAME} ${LIBNAME_LIST})

###############################################################################
# Install the library m3_controllers
install(TARGETS ${PROJECT_NAME} DESTINATION ${LIB_INSTALL_DIR})
