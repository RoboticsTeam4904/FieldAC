cmake_minimum_required(VERSION 3.9)
project(2018_field)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )

file(GLOB BOT_LCL_SRC botlocale/*.cpp)
file(GLOB BOT_TRK_SRC bottracking/*.cpp)
file(GLOB OBJ_TRK_SRC objecttracking/*.cpp)
file(GLOB CORE_SRC *.cpp)

add_executable(field ${CORE_SRC} ${BOT_LCL_SRC} ${BOT_TRK_SRC} ${OBJ_TRK_SRC})
#TARGET_LINK_LIBRARIES(field darknet-cpp-shared)

target_link_libraries(field ${OpenCV_LIBS})