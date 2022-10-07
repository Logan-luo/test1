cmake_minimum_required(VERSION 2.8)

project(DisplayImage)

set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_CXX_FLAGS "-Wall")

find_package(OpenCV REQUIRED)

add_executable(demo main.cpp)

target_include_directories(
    demo
    PUBLIC ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    demo
    ${OpenCV_LIBS}
)
