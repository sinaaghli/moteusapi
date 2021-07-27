cmake_minimum_required(VERSION 3.0)
project(Bar)

# find package moteus_cpp
find_package(moteus_cpp REQUIRED)
include_directories(${MOTEUS_CPP_INCLUDE_DIRS})

# build type
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
             "Debug"
             "Release"
             "MinSizeRel"
             "RelWithDebInfo"
)

# add binary
add_executable(bar bar.cpp)

# add library
target_link_libraries(bar ${MOTEUS_CPP_LIBRARIES})

