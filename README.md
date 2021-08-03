C++ API for Moteus BLDC driver/controller through fdcanusb
=====================

TODO:: reference the moteus product page and github

### How to compile?

Example of a local installation:

    > cd build
    > cmake ..
    > make install  # skip this step if you just want to run your code in example_internal folder

Installed files:

    > tree ../installed

    ├── bin
    │   └── tool
    ├── include
    │   └── foo
    │       ├── foo.h
    │       └── version.h
    └── lib
        ├── CMake
        │   └── Foo
        │       ├── FooConfig.cmake
        │       ├── FooConfigVersion.cmake
        │       ├── FooTargets.cmake
        │       └── FooTargets-noconfig.cmake
        └── libfoo.so

Uninstall library:

    > make uninstall

### How to use the library (as dependency) in an external project?

    cmake_minimum_required(VERSION 3.0)
    project(foo)

    find_package(moteus_cpp REQUIRED)
    include_directories(${MOTEUS_CPP_INCLUDE_DIRS})

    add_executable(foo foo.cpp)
    target_link_libraries(foo ${MOTEUS_CPP_LIBRARIES})

See the [example of external project](example_external/).

