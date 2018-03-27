cmake_minimum_required(VERSION 3.9)
project(2018_field)

set(CMAKE_CXX_STANDARD 11)

find_package( Threads )
find_package( OpenCV REQUIRED )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(DARKNET_GPU 1)
set(DARKNET_CUDNN 1)
set(DARKNET_DEBUG 0)
set(DARKNET_OPENMP 0)
set(DARKNET_LIBSO_NAME libdarknet.so)

include(DownloadProject.cmake)

file(GLOB NETWORK_SRC src/network/*.cpp)
file(GLOB BOT_LCL_SRC src/botlocale/*.cpp)
file(GLOB BOT_TRK_SRC src/bottracking/*.cpp)
file(GLOB OBJ_TRK_SRC src/objecttracking/*.cpp)
file(GLOB CORE_SRC src/*.cpp)

download_project(PROJ darknet
        GIT_REPOSITORY https://github.com/RoboticsTeam4904/darknet
        GIT_TAG botprov-compat
        GIT_SHALLOW 1)

download_project(PROJ rplidar
        URL https://download.slamtec.com/api/download/rplidar-sdk/1.5.7
        URL_MD5 e2a894e706f9ec6118967101c11d1dee
        DOWNLOAD_NAME rplidar-sdk.zip
        DOWNLOAD_NO_PROGRESS 1)

download_project(PROJ ntcore
        GIT_REPOSITORY https://github.com/wpilibsuite/ntcore
        GIT_TAG 3025a18
        GIT_SHALLOW 1)

download_project(PROJ ntcore-shared
        URL https://crate.botprovoking.org/fieldac/libntcore.so
        URL_MD5 3dc47662ddf949a2212fa0cd02265940
        DOWNLOAD_NAME libntcore.so
        DOWNLOAD_NO_EXTRACT 1
        DOWNLOAD_NO_PROGRESS 1)
#URL_MD5 7fcbd6a85e1034ea2eb53baecc5d49e0

download_project(PROJ wpiutil
        GIT_REPOSITORY https://github.com/wpilibsuite/wpiutil
        GIT_TAG 71d06a1
        GIT_SHALLOW 1)

download_project(PROJ wpiutil-shared
        URL https://crate.botprovoking.org/fieldac/libwpiutil.so
        URL_MD5 2e9e8f0634a247c4b559da18555ee8e6
        DOWNLOAD_NAME libwpiutil.so
        DOWNLOAD_NO_EXTRACT 1
        DOWNLOAD_NO_PROGRESS 1)
#URL_MD5 d22a2f7f0c7787cfa35caa7a704b0afe

set(rplidar_SDK_DIR ${rplidar_SOURCE_DIR}/sdk/sdk/)
execute_process(COMMAND uname -s
        OUTPUT_VARIABLE RPLIDAR_PLATFORM
        OUTPUT_STRIP_TRAILING_WHITESPACE)
set(rplidar_OUTPUT_DIR ${rplidar_SOURCE_DIR}/sdk/output/${RPLIDAR_PLATFORM}/Release/)

set(ntcore_GRADLE_DIR ${ntcore_SOURCE_DIR}/build/)
set(wpiutil_GRADLE_DIR ${wpiutil_SOURCE_DIR}/build/)

include_directories(${rplidar_SDK_DIR}/include/)
include_directories(${darknet_SOURCE_DIR}/src/)
include_directories(${wpiutil_SOURCE_DIR}/src/main/native/include/)
include_directories(${ntcore_SOURCE_DIR}/src/main/native/include/)
include_directories(${ntcore_SOURCE_DIR}/src/main/native/cpp/)
#include_directories(${nlohmann-json_SOURCE_DIR}/nlohmann/)

link_directories(${rplidar_OUTPUT_DIR})
link_directories(${CMAKE_BINARY_DIR})
link_directories(ntcore-shared-download/ntcore-shared-download-prefix/src/)
link_directories(wpiutil-shared-download/wpiutil-shared-download-prefix/src/)

MESSAGE(STATUS
        "CURR: "
        ${CMAKE_BINARY_DIR})

add_executable(field ${CORE_SRC}
        ${BOT_LCL_SRC}
        ${BOT_TRK_SRC}
        ${OBJ_TRK_SRC}
        ${NETWORK_SRC})

add_custom_command(TARGET field
        PRE_LINK
        COMMAND cp ntcore-shared-download/ntcore-shared-download-prefix/src/libntcore.so ${CMAKE_BINARY_DIR}/
        COMMENT "Copying ntcore..."
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_custom_command(TARGET field
        PRE_LINK
        COMMAND cp wpiutil-shared-download/wpiutil-shared-download-prefix/src/libwpiutil.so ${CMAKE_BINARY_DIR}/
        COMMENT "Copying wpiutil..."
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_custom_command(TARGET field
        PRE_LINK
        COMMAND make obj libdarknet.so GPU=${DARKNET_GPU} CUDNN=${DARKNET_CUDNN} OPENCV=1 TRACK_OPTFLOW=1 DEBUG=${DARKNET_DEBUG} OPENMP=${DARKNET_OPEPNMP} LIBSO=1 LIBNAMESO=${DARKNET_LIBSO_NAME}
        COMMAND cp ${DARKNET_LIBSO_NAME} ${CMAKE_BINARY_DIR}/
        COMMENT "Building ${DARKNET_LIBSO_NAME}..."
        BYPRODUCTS ${CMAKE_BINARY_DIR}/${DARKNET_LIBSO_NAME}
        WORKING_DIRECTORY ${darknet_SOURCE_DIR})

add_custom_command(TARGET field
        PRE_LINK
        COMMAND make build_sdk
        WORKING_DIRECTORY ${rplidar_SDK_DIR}
        COMMENT "Building ${rplidar_SDK_DIR}"
        BYPRODUCTS ${rplidar_OUTPUT_DIR}/librplidar_sdk.a)

target_link_libraries(field ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(field rplidar_sdk)
target_link_libraries(field darknet)
target_link_libraries(field ntcore)
target_link_libraries(field ${OpenCV_LIBS})

