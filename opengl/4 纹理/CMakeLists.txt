cmake_minimum_required(VERSION 3.25)
project(4)
set(CMAKE_CXX_STANDARD 17)
include_directories(./include)

find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(glfw3 REQUIRED)

add_executable(4 src/main.cc src/glad.c include/stb_image.h include/learnopengl/shader_s.h)

target_link_libraries(4 GL glfw)
