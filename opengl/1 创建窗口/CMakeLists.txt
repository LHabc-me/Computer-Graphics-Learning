cmake_minimum_required(VERSION 3.25)
project(1)
set(CMAKE_CXX_STANDARD 17)
include_directories(./include)

find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(glfw3 REQUIRED)

add_executable(1 src/main.cc src/glad.c)

target_link_libraries(1 GL glfw)
