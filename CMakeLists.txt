cmake_minimum_required(VERSION 3.9)
project(2018_field)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

file(GLOB ARAPAHO_SRC arapaho/*.cpp)
file(GLOB NETWORK_SRC network/*.cpp)
file(GLOB BOT_LCL_SRC botlocale/*.cpp)
file(GLOB BOT_TRK_SRC bottracking/*.cpp)
file(GLOB OBJ_TRK_SRC objecttracking/*.cpp)
file(GLOB CORE_SRC *.cpp)

file(GLOB DARKNET_INC darknet/*.h*)

link_directories(include/)
include_directories(${CMAKE_SOURCE_DIR}/darknet/)

add_executable(field ${DARKNET_INC}
        ${CORE_SRC}
        ${BOT_LCL_SRC}
        ${BOT_TRK_SRC}
        ${OBJ_TRK_SRC}
        ${NETWORK_SRC}
        ${ARAPAHO_SRC})
target_link_libraries(field darknet)
target_link_libraries(field ${OpenCV_LIBS})