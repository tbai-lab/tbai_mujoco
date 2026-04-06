cmake_minimum_required(VERSION 3.22)
project(tbai_mujoco_examples CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# ── tbai_sdk (brings in zenohc / zenohcxx transitively) ──────────────────
find_package(tbai_sdk REQUIRED)

# lodepng (for depth heatmap PNG encoding)
set(LODEPNG_DIR "${CMAKE_SOURCE_DIR}/simulate/src/lodepng")

add_executable(stand_go2 stand_go2.cpp ${LODEPNG_DIR}/lodepng.cpp)
target_include_directories(stand_go2 PRIVATE ${LODEPNG_DIR})
target_link_libraries(stand_go2 tbai_sdk::tbai_sdk)
