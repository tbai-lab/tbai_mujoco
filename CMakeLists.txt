cmake_minimum_required(VERSION 3.22)
project(tbai_mujoco)

add_subdirectory(simulate)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/example)
  add_subdirectory(example)
endif()
