#Specify the minimum version of cmake to ensure that certain new features can be used to build the project
cmake_minimum_required(VERSION 3.4.1)

#Output detailed information
set(CMAKE_VERBOSE_MAKEFILE on)

#Set the header file directory
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/src/main/cpp/include")
#Set the source file directory
set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/src/main/cpp/source")
#Set library directory
set(LIBRARY_DIR "${CMAKE_SOURCE_DIR}/src/main/jniLibs")

#Include header file
include_directories(${INCLUDE_DIR})

#Add fastcv static library
add_library(fastcv-lib STATIC IMPORTED)
set_target_properties( fastcv-lib
                       PROPERTIES IMPORTED_LOCATION
                       "${LIBRARY_DIR}/libfastcv.a"
                      )

#Add log library
find_library( # Sets the name of the path variable.
                log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

find_path(GLES2_INCLUDE_DIR GLES2/gl2.h
    HINTS ${ANDROID_NDK})
include_directories(${GLES2_INCLUDE_DIR})

find_library(GLES2_LIBRARY libGLESv2.so
    HINTS ${GLES2_INCLUDE_DIR}/../lib)

#Include subdirectories
add_subdirectory(${SOURCE_DIR})
