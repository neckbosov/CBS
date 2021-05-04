cmake_minimum_required(VERSION 3.19)
project(course_project)

set(CMAKE_CXX_STANDARD 17)

add_executable(course_project main.cpp graph.h astar.h astar.cpp graph.cpp)