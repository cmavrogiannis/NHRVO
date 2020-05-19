cmake_minimum_required(VERSION 3.10.2)

project(NHRVO LANGUAGES CXX)

add_subdirectory(RVO2)
add_subdirectory(Dubins-Curves)

file(GLOB src_files
  "src/*.h"
  "src/*.cpp")

add_executable(NHRVO ${src_files})

if(OpenMP_CXX_FOUND)
  target_link_libraries(NHRVO OpenMP::OpenMP_CXX)
endif()

include_directories(
  RVO2/src
  Dubins-Curves/include)

target_link_libraries(NHRVO
  RVO
  dubins)
