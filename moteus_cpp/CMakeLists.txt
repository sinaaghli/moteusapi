# Set SOURCES variable
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/mjbots/moteus/*.cpp
                  ${CMAKE_CURRENT_SOURCE_DIR}/mjbots/pi3hat/*.cpp)

# Set HEADERS variable
file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/mjbots/moteus/*.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/mjbots/pi3hat/*.h)

include(${CMAKE_SOURCE_DIR}/cmake/LibraryConfig.cmake)
