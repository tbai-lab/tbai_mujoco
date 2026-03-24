cmake_minimum_required(VERSION 3.22)
project(tbai_mujoco_examples CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# ── tbai_sdk ──────────────────────────────────────────────────────────────
set(TBAI_SDK_DIR "/home/kb/projects/tbai_sdk" CACHE PATH "Path to tbai_sdk")
set(ZENOH_INSTALL "${TBAI_SDK_DIR}/deps/zenoh-install")

# lodepng (for depth heatmap PNG encoding)
set(LODEPNG_DIR "${CMAKE_SOURCE_DIR}/simulate/src/lodepng")

add_executable(stand_go2 stand_go2.cpp ${LODEPNG_DIR}/lodepng.cpp)
target_include_directories(stand_go2 PRIVATE
  ${TBAI_SDK_DIR}/include
  ${ZENOH_INSTALL}/include
  ${LODEPNG_DIR}
)
target_compile_definitions(stand_go2 PRIVATE ZENOHCXX_ZENOHC)
target_link_directories(stand_go2 PRIVATE ${ZENOH_INSTALL}/lib)
target_link_libraries(stand_go2 zenohc pthread)
