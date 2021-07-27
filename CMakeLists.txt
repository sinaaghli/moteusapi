cmake_minimum_required(VERSION 3.0)
project(moteus_cpp)

# set default build to debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release or Debug" FORCE)
endif(NOT CMAKE_BUILD_TYPE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall" CACHE INTERNAL "")

# find library dependencies (comment if there are )
# FIND_PACKAGE( DUMMYDEP REQUIRED )

# SET(DEP_INCLUDE_DIRS 
#       ${DUMMYDEP_INCLUDE_DIR})

# SET(DEP_LIBS 
#       ${DUMMYDEP_LIBRARIES})

# Set variables
include(${CMAKE_SOURCE_DIR}/cmake/SetEnv.cmake)

# Library sources
add_subdirectory(${LIBRARY_FOLDER})

# Library examples
add_subdirectory(example_internal)

# Install targets
include(${CMAKE_SOURCE_DIR}/cmake/InstallConfig.cmake)
