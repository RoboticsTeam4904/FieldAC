cmake_minimum_required(VERSION 3.9)
project(2018_field)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

file(GLOB DARKNET_SRC x12/*.c*)
file(GLOB BOT_LCL_SRC botlocale/*.cpp)
file(GLOB BOT_TRK_SRC bottracking/*.cpp)
file(GLOB OBJ_TRK_SRC objecttracking/*.cpp)
file(GLOB CORE_SRC *.cpp)

link_directories(include/)
add_executable(field ${CORE_SRC} ${BOT_LCL_SRC} ${BOT_TRK_SRC} ${OBJ_TRK_SRC} ${DARKNET_SRC})
#target_include_directories(field PUBLIC include)
target_link_libraries(field darknet-cpp-shared)
target_link_libraries(field ${OpenCV_LIBS})