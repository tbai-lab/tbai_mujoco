cmake_minimum_required(VERSION 3.22)
project(tbai_mujoco_simulate CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
include(GNUInstallDirs)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -fPIC")
endif()

# ── MuJoCo (pre-built release) ────────────────────────────────────────────
set(MUJOCO_VERSION "3.6.0")
set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz")

FetchContent_Declare(
  mujoco
  URL ${MUJOCO_URL}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(mujoco)

set(MUJOCO_DIR ${mujoco_SOURCE_DIR})
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIB_DIR ${MUJOCO_DIR}/lib)
set(MUJOCO_LIB ${MUJOCO_LIB_DIR}/libmujoco.so.${MUJOCO_VERSION})

message(STATUS "MuJoCo source dir: ${MUJOCO_DIR}")
message(STATUS "MuJoCo include dir: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo lib: ${MUJOCO_LIB}")

# ── tbai_sdk ──────────────────────────────────────────────────────────────
set(TBAI_SDK_DIR "${CMAKE_SOURCE_DIR}/deps/tbai_sdk" CACHE PATH "Path to tbai_sdk")
if(NOT EXISTS "${TBAI_SDK_DIR}/include/tbai_sdk/session.hpp")
  message(FATAL_ERROR "tbai_sdk not found at ${TBAI_SDK_DIR}.\nRun: pixi run setup-tbai-sdk")
endif()
message(STATUS "tbai_sdk dir: ${TBAI_SDK_DIR}")

set(ZENOH_INSTALL "${TBAI_SDK_DIR}/deps/zenoh-install" CACHE PATH "Zenoh install prefix")
if(NOT EXISTS "${ZENOH_INSTALL}/lib/libzenohc.so")
  message(FATAL_ERROR
    "Zenoh not found at ${ZENOH_INSTALL}.\n"
    "Run: pixi run setup-tbai-sdk")
endif()

# ── Dependencies ──────────────────────────────────────────────────────────
include_directories(
  ${MUJOCO_INCLUDE_DIR}
  ${MUJOCO_INCLUDE_DIR}/mujoco
  ${MUJOCO_DIR}/simulate
  ${TBAI_SDK_DIR}/include
  ${ZENOH_INSTALL}/include
  src/lodepng
)

file(GLOB SIM_SRC
  src/lodepng/lodepng.cpp
  ${MUJOCO_DIR}/simulate/glfw_*.cc
  ${MUJOCO_DIR}/simulate/platform_*.cc
  ${MUJOCO_DIR}/simulate/simulate.cc
)

add_executable(tbai_mujoco ${SIM_SRC} src/main.cc)
target_compile_definitions(tbai_mujoco PRIVATE ZENOHCXX_ZENOHC)
target_link_directories(tbai_mujoco PRIVATE ${ZENOH_INSTALL}/lib)
target_link_libraries(tbai_mujoco
  ${MUJOCO_LIB}
  pthread
  glfw
  yaml-cpp
  zenohc
)

# ── Copy MuJoCo plugins next to the binary so they are auto-loaded ────────
add_custom_command(TARGET tbai_mujoco POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${MUJOCO_DIR}/bin/mujoco_plugin"
    "$<TARGET_FILE_DIR:tbai_mujoco>/mujoco_plugin"
  COMMENT "Copying MuJoCo plugins"
)

# ── Install ───────────────────────────────────────────────────────────────
install(DIRECTORY ${MUJOCO_DIR}/bin/
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  FILES_MATCHING PATTERN "*"
)

install(DIRECTORY ${MUJOCO_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${MUJOCO_DIR}/lib/
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  FILES_MATCHING PATTERN "*.so*" PATTERN "*.a"
)

install(TARGETS tbai_mujoco
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
