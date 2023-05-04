###########################
#                         #
#          Title          #
#                         #
###########################

cmake_minimum_required(VERSION 3.10)

project(TestRun LANGUAGES CXX)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE "Debug")
#set(BUILD_TYPE)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../../bin/x86)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../../lib/x86)
message("LIBRARY_OUTPUT_PATH = " ${LIBRARY_OUTPUT_PATH})
message("EXECUTABLE_OUTPUT_PATH = " ${EXECUTABLE_OUTPUT_PATH})

function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})
        message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif ()
    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*)
    foreach (sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub})
        endif ()
    endforeach ()
endfunction()

###########################
#                         #
#        VERSION          #
#                         #
###########################


set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)

configure_file(
        ${CMAKE_SOURCE_DIR}/src/version.h.in
        ${CMAKE_SOURCE_DIR}/include/version.h
        @ONLY
)


###########################
#                         #
#        OpenCV           #
#                         #
###########################

# 20.04 注释
if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    #elseif (BUILD_TYPE STREQUAL "x86")
    set(CMAKE_PREFIX_PATH "/home/whb/dockerspace/lib/x86_opencv")
endif ()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

###########################
#                         #
#          PCL            #
#                         #
###########################

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${PROJECT_SOURCE_DIR}/lib)

##############################
#                            #
#           OpenMP           #
#                            #
##############################

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    message(STATUS "found openmp")
#    set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS}, ${OPENMP_C_FLAGS})
#    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}, ${OPENMP_CXX_FLAGS})
elseif()
    message(FATAL_ERROR "openmp not found!")
endif ()
###########################
#                         #
#          RCS            #
#                         #
###########################
set(BUILD_TYPE)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../../bin/x86)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../../lib/x86)
link_directories(${LIBRARY_OUTPUT_PATH})

include_directories(
        ${PROJECT_SOURCE_DIR}/../../message
        ${PROJECT_SOURCE_DIR}/../shpobjlib
        ${PROJECT_SOURCE_DIR}/Common
        ${PROJECT_SOURCE_DIR}/transform
        ${PROJECT_SOURCE_DIR}/nml_msg
        ${PROJECT_SOURCE_DIR}/nml_server
        ${PROJECT_SOURCE_DIR}/
        ${PROJECT_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIRS}

)
link_directories(
        /usr/local/lib
        ${PROJECT_SOURCE_DIR}/lib

)
############################
##                         #
##          other           #
##                         #
############################
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system thread filesystem)

include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/include)

include_directories(
        ${PROJECT_SOURCE_DIR}/../../message
        ${PROJECT_SOURCE_DIR}/../shpobjlib
        ${PROJECT_SOURCE_DIR}/common
        ${PROJECT_SOURCE_DIR}/transform
        ${PROJECT_SOURCE_DIR}/
        ${PROJECT_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../xsproto_include
        /usr/local/include
)

# add_subdirectory(src/ndt)

file(GLOB SRC_FILES
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        #  "${PROJECT_SOURCE_DIR}/common/*.cpp"
        #  "${PROJECT_SOURCE_DIR}/transform/*.cpp"
        )

#link_directories(
#    ${LIBRARY_OUTPUT_PATH}
#    ${PROJECT_SOURCE_DIR}/lib
#    /usr/local/lib
#    /usr/lib
#    ${PROJECT_SOURCE_DIR}/../../../lib
#    )

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}
        -lpthread
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        librcs.a -lpthread libUGVAuto.a libshpobjd.a libNmlMsg.a xsfault ndt_matcher # rcs


#        -lboost_system -lboost_serialization #Common
        #        libshpobjd.a
        #        xsfault
        #        protobuf
        #        ndt_matcher
        #        boost_serialization
        #        fastrtps
        #        fastcdr
        #        xscom
        #        libxsproto.a
        )

