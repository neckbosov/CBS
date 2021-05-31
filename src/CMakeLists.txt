set(BINARY ${CMAKE_PROJECT_NAME})
find_package(Boost 1.70 REQUIRED)
message(STATUS "Boost version: ${Boost_VERSION}")

file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true *.h *.cpp)

set(SOURCES ${SOURCES})

add_executable(${BINARY}_run ${SOURCES})

add_library(${BINARY}_lib STATIC ${SOURCES})

target_link_libraries(${BINARY}_lib ${Boost_LIBRARIES})