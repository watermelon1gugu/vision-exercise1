cmake_minimum_required(VERSION 3.8)
project(vision)

set(CMAKE_CXX_STANDARD 11)



set(SOURCE_FILES main.cpp)
add_executable(vision ${SOURCE_FILES})

find_package(OpenCV REQUIRED)
target_link_libraries(vision ${OpenCV_LIBS})