cmake_minimum_required(VERSION 3.9)
project(2018_field)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_VERBOSE_MAKEFILE ON)

find_package( OpenCV REQUIRED )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(DownloadProject.cmake)

download_project(PROJ darknet
        GIT_REPOSITORY https://github.com/AlexeyAB/darknet
        GIT_TAG 51d99f5)

file(GLOB NETWORK_SRC network/*.cpp)
file(GLOB BOT_LCL_SRC botlocale/*.cpp)
file(GLOB BOT_TRK_SRC bottracking/*.cpp)
file(GLOB OBJ_TRK_SRC objecttracking/*.cpp)
file(GLOB CORE_SRC *.cpp)

include_directories(${darknet_SOURCE_DIR}/src/)
link_directories(include/)

add_executable(field ${CORE_SRC}
        ${BOT_LCL_SRC}
        ${BOT_TRK_SRC}
        ${OBJ_TRK_SRC}
        ${NETWORK_SRC})

add_custom_command(TARGET field
        PRE_LINK
        COMMAND make obj libdarknet.so GPU=0 CUDNN=0 OPENCV=1 DEBUG=1 OPENMP=0 LIBSO=1 LIBNAMESO=libdarknet.so
        COMMAND cp libdarknet.so ${CMAKE_SOURCE_DIR}/include/
        COMMAND cp libdarknet.so ${CMAKE_BINARY_DIR}/
        WORKING_DIRECTORY ${darknet_SOURCE_DIR})

target_link_libraries(field darknet)
target_link_libraries(field ${OpenCV_LIBS})

