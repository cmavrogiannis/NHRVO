cmake_minimum_required(VERSION 3.10.2)

project(NHRVO LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(RVO2)

add_executable(global_planner "src/global_planner.cpp"
  "src/NHRVO.h")
add_executable(local_planner "src/local_planner.cpp"
  "src/NHRVO.h")
add_executable(holonomic "src/holonomic.cpp"
  "src/NHRVO.h")
add_executable(pid "src/pid.cpp"
  "src/NHRVO.h")

if(OpenMP_CXX_FOUND)
  target_link_libraries(global_planner OpenMP::OpenMP_CXX)
  target_link_libraries(local_planner OpenMP::OpenMP_CXX)
  target_link_libraries(holonomic OpenMP::OpenMP_CXX)
  target_link_libraries(pid OpenMP::OpenMP_CXX)
endif()

include_directories(
  RVO2/src
  Dubins-Curves/include
  Dubins-Curves/src)

target_link_libraries(global_planner
  RVO)

target_link_libraries(local_planner
  RVO)

target_link_libraries(holonomic
  RVO)

target_link_libraries(pid
  RVO)

add_custom_target(visualize_global
  COMMAND ./global_planner | python3 ${CMAKE_SOURCE_DIR}/visualize.py
  VERBATIM)

add_custom_target(visualize_local
  COMMAND ./local_planner | python3 ${CMAKE_SOURCE_DIR}/visualize.py
  VERBATIM)

add_custom_target(visualize_holonomic
  COMMAND ./holonomic | python3 ${CMAKE_SOURCE_DIR}/visualize.py
  VERBATIM)

add_custom_target(visualize_pid
  COMMAND ./pid | python3 ${CMAKE_SOURCE_DIR}/visualize.py
  VERBATIM)

add_dependencies(visualize_global global_planner)
add_dependencies(visualize_local local_planner)
add_dependencies(visualize_holonomic holonomic)
add_dependencies(visualize_pid pid)
