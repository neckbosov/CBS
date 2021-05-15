cmake_minimum_required(VERSION 3.19)
project(course_project)

link_libraries(gtest)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

add_executable(course_project main.cpp graph.h astar.h cbs.h cbs.cpp task.cpp task.h map.h test/test_astar.cpp)


enable_testing()
add_executable(mytest test/test_astar.cpp)
target_link_libraries(mytest cppunit)
add_test(NAME mytest COMMAND mytest)