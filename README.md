C++ API for Moteus BLDC controller through fdcanusb
=====================

This is a c++ API to communicate with [MJBOT](https://mjbots.com/)'s [Moteus](https://github.com/mjbots/moteus) brushless dc motor driver through [fdcanusb](https://mjbots.com/collections/accessories/products/fdcanusb) connected to a Mac or Linux PC.

## To try the API without system wide installation?
Compile and run the examples at [example_internal](example_internal/) folder

to compile try

    > cd build
    > cmake ..
    > make
    > sudo ./example_internal/positioncmd


## Compile and install in your system

You can install moteusapi on your system and then have a copy of the folder [example_external](example_external/) somewhere on your system. running cmake on example_external would be able to find the library on your system.

first comile and install

    > cd build
    > cmake ..
    > make install 

then compile eample_external

    > cp -r moteusapi/example_external /your/code/folder/
    > cd /your/code/folder/example_external
    > mkdir build
    > cd build
    > cmake ..
    > make
    > sudo ./positioncmd


Uninstall library:

    > make uninstall

## How to use the library (as dependency) in an external project?

here is a snippet for your cmake project,

    > cmake_minimum_required(VERSION 3.0)
    > project(foo)
    > find_package(MoteusAPI REQUIRED)
    > include_directories(${MOTEUSAPI_INCLUDE_DIRS})
    > add_executable(foo foo.cpp)
    > target_link_libraries(foo ${MOTEUSAPI_LIBRARIES})

See the [example of external project](example_external/).

## LICENSE
All files contained in this repository, unless otherwise noted, are available under an Apache 2.0 License: https://www.apache.org/licenses/LICENSE-2.0

