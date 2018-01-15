cmake_minimum_required(VERSION 3.9)
project(2018_field)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )

### Darknet External Project

#set(DARKNETCPP_ROOT ${CMAKE_BINARY_DIR}/darknetcpp)
#set(DARKNETCPP_LIB ${DARKNETCPP_ROOT}/src/libdarknet)
#set(DARKNETCPP_ARAPAHO ${DARKNETCPP_ROOT}/src/libdarknet/arapaho)
#
#include(ExternalProject)
#
#ExternalProject_Add(libdarknet-ext
#        PREFIX ${DARKNETCPP_ROOT}
#        GIT_REPOSITORY "https://github.com/prabindh/darknet"
#        GIT_TAG "master"
#        CONFIGURE_COMMAND ""
#        SOURCE_DIR ${DARKNETCPP_ROOT}/src/libdarknet
#        BUILD_COMMAND make darknet-cpp-shared GPU=0 CUDNN=0 OPENCV=1
#        BUILD_IN_SOURCE 1
#        INSTALL_COMMAND ""
#        TEST_COMMAND ""
##        BUILD_BYPRODUCTS libdarknet-cpp-shared.so
#)
##
#ExternalProject_Add_Step(libdarknet-ext
#        move
#        COMMAND mv ${DARKNETCPP_LIB}/libdarknet-cpp-shared.so ${CMAKE_SOURCE_DIR}/darknet
#        DEPENDEES build
#        WORKING_DIRECTORY ${DARKNETCPP_ARAPAHO}/
#)
#
#add_library(darknetcpp SHARED IMPORTED)
#set_target_properties(darknetcpp PROPERTIES IMPORTED_LOCATION ${DARKNETCPP_LIB}/libdarknet-cpp-shared.so)
#add_dependencies(darknetcpp libdarknet-ext)

### End of Darknet Project

file(GLOB BOT_LCL_SRC botlocale/*.cpp)
file(GLOB BOT_TRK_SRC bottracking/*.cpp)
file(GLOB OBJ_TRK_SRC objecttracking/*.cpp)
file(GLOB DARKNET_SRC darknet/*.c*)

file(GLOB CORE_SRC *.cpp)

#include_directories(${CMAKE_SOURCE_DIR}/darknet)
#link_directories(${CMAKE_SOURCE_DIR}/darknet)


add_executable(field ${CORE_SRC} ${BOT_LCL_SRC} ${BOT_TRK_SRC} ${OBJ_TRK_SRC} ${DARKNET_SRC})
#TARGET_LINK_LIBRARIES(field darknet-cpp-shared)

target_link_libraries(field ${OpenCV_LIBS})