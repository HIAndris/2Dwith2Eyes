cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
list(APPEND EXTRA_COMPONENT_DIRS .pio/libdeps/esp32-s3-devkitc-1/esp32-camera)
project(2Dwith2Eyes)
