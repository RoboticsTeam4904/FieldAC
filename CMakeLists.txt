cmake_minimum_required(VERSION 3.9)
project(2018_field)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(DARKNET_GPU 0)
set(DARKNET_CUDNN 0)
set(DARKNET_DEBUG 0)
set(DARKNET_OPENMP 0)
set(DARKNET_LIBSO_NAME libdarknet.so)

include(DownloadProject.cmake)

file(GLOB NETWORK_SRC network/*.cpp)
file(GLOB BOT_LCL_SRC botlocale/*.cpp)
file(GLOB BOT_TRK_SRC bottracking/*.cpp)
file(GLOB OBJ_TRK_SRC objecttracking/*.cpp)
file(GLOB CORE_SRC *.cpp)

download_project(PROJ darknet
        GIT_REPOSITORY https://github.com/HowardStark/darknet
        GIT_TAG master)

download_project(PROJ rplidar
        URL https://download.slamtec.com/api/download/rplidar-sdk/1.5.7
        URL_MD5 e2a894e706f9ec6118967101c11d1dee
        DOWNLOAD_NAME rplidar-sdk.zip
        DOWNLOAD_NO_PROGRESS 1)

download_project(PROJ nlohmann-json
        URL https://github.com/nlohmann/json/releases/download/v3.1.0/include.zip
        URL_MD5 a88e64482a9a6f2222d7cc79df5790e1
        DOWNLOAD_NAME nlohmann-json.zip
        DOWNLOAD_NO_PROGRESS 1)

set(rplidar_SDK_DIR ${rplidar_SOURCE_DIR}/sdk/sdk/)
execute_process(COMMAND uname -s
        OUTPUT_VARIABLE RPLIDAR_PLATFORM
        OUTPUT_STRIP_TRAILING_WHITESPACE)
set(rplidar_OUTPUT_DIR ${rplidar_SOURCE_DIR}/sdk/output/${RPLIDAR_PLATFORM}/Release/)

include_directories(${rplidar_SDK_DIR}/include/)
include_directories(${darknet_SOURCE_DIR}/src/)
include_directories(${nlohmann-json_SOURCE_DIR}/nlohmann/)
link_directories(${rplidar_OUTPUT_DIR})
link_directories(${CMAKE_BINARY_DIR})

add_executable(field ${CORE_SRC}
        ${BOT_LCL_SRC}
        ${BOT_TRK_SRC}
        ${OBJ_TRK_SRC}
        ${NETWORK_SRC} objecttracking/extrapolate.cpp objecttracking/extrapolate.h)

add_custom_command(TARGET field
        PRE_LINK
        COMMAND make obj libdarknet.so GPU=0 CUDNN=${DARKNET_CUDNN} OPENCV=1 TRACK_OPTFLOW=1 DEBUG=${DARKNET_DEBUG} OPENMP=${DARKNET_OPEPNMP} LIBSO=1 LIBNAMESO=${DARKNET_LIBSO_NAME}
        COMMAND cp ${DARKNET_LIBSO_NAME} ${CMAKE_BINARY_DIR}/
        COMMENT "Building ${DARKNET_LIBSO_NAME}..."
        BYPRODUCTS ${CMAKE_SOURCE_DIR}/include/${DARKNET_LIBSO_NAME} ${CMAKE_BINARY_DIR}/${DARKNET_LIBSO_NAME}
        WORKING_DIRECTORY ${darknet_SOURCE_DIR})

add_custom_command(TARGET field
        PRE_LINK
        COMMAND make build_sdk
        WORKING_DIRECTORY ${rplidar_SDK_DIR}
        COMMENT "Building ${rplidar_SDK_DIR}"
        BYPRODUCTS ${rplidar_OUTPUT_DIR}/librplidar_sdk.a)

target_link_libraries(field rplidar_sdk)
target_link_libraries(field darknet)
target_link_libraries(field ${OpenCV_LIBS})

