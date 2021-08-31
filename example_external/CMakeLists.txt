cmake_minimum_required(VERSION 3.0)
project(MyMoteusProject)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wfatal-errors -Wall" CACHE INTERNAL "")

# find package moteus_cpp
find_package(MoteusAPI REQUIRED)
include_directories(${MOTEUSAPI_INCLUDE_DIRS})

# build type
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
             "Debug"
             "Release"
             "MinSizeRel"
             "RelWithDebInfo"
)

# add binary
add_executable(positioncmd main_positioncmd.cpp)

# link to library
target_link_libraries(positioncmd ${MOTEUSAPI_LIBRARIES})

