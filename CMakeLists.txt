cmake_minimum_required(VERSION 3.10.2)

project(NHRVO LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(RVO2)

file(GLOB src_files
  "src/*.h"
  "src/*.cpp")

add_executable(NHRVO ${src_files})

if(OpenMP_CXX_FOUND)
  target_link_libraries(NHRVO OpenMP::OpenMP_CXX)
endif()

include_directories(
  RVO2/src
  Dubins-Curves/include
  Dubins-Curves/src)

target_link_libraries(NHRVO
  RVO)

add_custom_target(visualize
  COMMAND ./NHRVO | python3 ${CMAKE_SOURCE_DIR}/visualize.py
  VERBATIM)

add_dependencies(visualize NHRVO)
