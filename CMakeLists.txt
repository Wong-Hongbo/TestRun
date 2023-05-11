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

############################
##                         #
##          other          #
##                         #
############################

link_directories(
        ${PROJECT_SOURCE_DIR}/../../../lib/x86
        /usr/local/lib
        ${PROJECT_SOURCE_DIR}/lib
)

include_directories(
        ${PROJECT_SOURCE_DIR}/../../xsnml_include
        ${PROJECT_SOURCE_DIR}/../../xsmap/nml_msg
        ${PROJECT_SOURCE_DIR}/../../xsproto_include
        ${PROJECT_SOURCE_DIR}/../../fault_include

        ${PROJECT_SOURCE_DIR}/../shpobjlib
        ${PROJECT_SOURCE_DIR}/common
        ${PROJECT_SOURCE_DIR}/transform
        ${PROJECT_SOURCE_DIR}/
        ${PROJECT_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        /usr/local/include
)
include_sub_directories_recursively(
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/common
        )

file(GLOB SRC_FILES
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        "${PROJECT_SOURCE_DIR}/common/transform/*.cpp"
        "${PROJECT_SOURCE_DIR}/common/common/*.cpp"
        #  "${PROJECT_SOURCE_DIR}/transform/*.cpp"
        )
add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}
        -lpthread
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}

#        libUGVAuto.a
        libshpobjd.a
        libNmlMsg.a
        fastrtps fastcdr xsfault
#        ndt_matcher # rcs
        rcs xscom boost_serialization
        )

