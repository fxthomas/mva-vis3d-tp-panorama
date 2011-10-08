CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} $ENV{IMAGINEPP_ROOT}/CMake)
SET(EXECUTABLE_OUTPUT_PATH ../)
FIND_PACKAGE(Imagine)

PROJECT(Panorama)

ImagineAddExecutable(Panorama ../src/Panorama.cpp)
ImagineUseModules(Panorama LinAlg Graphics Images)
