cmake_minimum_required(VERSION 2.8)
project(common)

set( CMAKE_BUILD_TYPE "Release" )

file(GLOB SDK_SRC "${PROJECT_SOURCE_DIR}/*.cpp")
add_library( common SHARED ${SDK_SRC})