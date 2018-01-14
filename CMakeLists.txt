cmake_minimum_required(VERSION 3.9)
project(2018-field)

find_package( OpenCV REQUIRED )

file(GLOB_RECURSE SRC_FILES *.cpp)
add_executable(field ${SRC_FILES})
target_link_libraries(field ${OpenCV_LIBS})