cmake_minimum_required(VERSION 3.9)
project(2018_field)

set(CMAKE_CXX_STANDARD 11)

find_package( Threads )
find_package( OpenCV REQUIRED )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(DARKNET_GPU 0)
set(DARKNET_CUDNN 0)
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

#download_project(PROJ ntcore-dylib
#        URL https://crate.botprovoking.org/fieldac/libntcore.macos-10.13.1.dylib
#        DOWNLOAD_NAME libntcore.dylib
#        DOWNLOAD_NO_EXTRACT 1
#        URL_MD5 de762fbcf34495bb5a06d1acec5f9767
#        DOWNLOAD_NO_PROGRESS 1)

download_project(PROJ ntcore
        GIT_REPOSITORY https://github.com/PeterJohnson/ntcore
        GIT_TAG nativearm
        GIT_SHALLOW 1)

download_project(PROJ wpiutil
        GIT_REPOSITORY https://github.com/wpilibsuite/wpiutil
        GIT_TAG 71d06a1
        GIT_SHALLOW 1)

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

add_executable(field ${CORE_SRC}
        ${BOT_LCL_SRC}
        ${BOT_TRK_SRC}
        ${OBJ_TRK_SRC}
        ${NETWORK_SRC})

add_custom_command(TARGET field
        PRE_LINK
        COMMAND make obj libdarknet.so GPU=1 CUDNN=${DARKNET_CUDNN} OPENCV=1 TRACK_OPTFLOW=1 DEBUG=${DARKNET_DEBUG} OPENMP=${DARKNET_OPEPNMP} LIBSO=1 LIBNAMESO=${DARKNET_LIBSO_NAME}
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

add_custom_command(TARGET field
        PRE_BUILD
        COMMAND ./gradlew build -PskipAthena -PskipAllTests
        COMMAND cp ${ntcore_GRADLE_DIR}/libs/ntcore/shared/libntcore.dylib ${CMAKE_BINARY_DIR}/
        COMMENT "Building ntcore..."
        WORKING_DIRECTORY ${ntcore_SOURCE_DIR})

add_custom_command(TARGET field
        PRE_BUILD
        COMMAND ./gradlew build -PskipAthena -PskipAllTests
        COMMAND cp ${wpiutil_GRADLE_DIR}/libs/wpiutil/shared/libwpiutil.dylib ${CMAKE_BINARY_DIR}/
        COMMENT "Building wpiutil..."
        WORKING_DIRECTORY ${wpiutil_SOURCE_DIR})

target_link_libraries(field ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(field rplidar_sdk)
target_link_libraries(field darknet)
target_link_libraries(field ntcore)
target_link_libraries(field ${OpenCV_LIBS})
