cmake_minimum_required(VERSION 3.19)
project(course_project)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

add_executable(course_project main.cpp graph.h astar.h ecbs.h ecbs.cpp cbs.h cbs.cpp)