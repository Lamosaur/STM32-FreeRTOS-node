cmake_minimum_required(VERSION 3.20)

project(stm32_sensor_node LANGUAGES C CXX ASM)

# Two build trees come out of this file:
#
#   Host (no toolchain file) — compiles the target-independent application core and the
#   unit tests. This is what FR-0.6 requires and what CI runs.
#       cmake -B build -G Ninja && cmake --build build && ctest --test-dir build
#
#   Target (cmake/arm-none-eabi.cmake) — compiles the same application core plus the
#   FreeRTOS port and the STM32F103 HAL, and links a flashable image.
#       cmake -B build-arm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#
# The application core is the same source list in both, which is the point: if it stops
# compiling on the host, either the C HAL / C++ application boundary has been broken or
# target-specific code has leaked out of firmware/hal and firmware/rtos.

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # The two trees want opposite defaults. The host tree wants Debug: it runs sanitizers
  # and the tests read better with symbols. The target tree wants MinSizeRel: a firmware
  # image is normally built for size, and on a 64 KB part an unoptimised Debug build is
  # large enough to change whether the project fits at all, which is not a decision to
  # make by accident.
  if(CMAKE_CROSSCOMPILING)
    set(_node_default_build_type MinSizeRel)
  else()
    set(_node_default_build_type Debug)
  endif()
  set(CMAKE_BUILD_TYPE ${_node_default_build_type} CACHE STRING "Build type" FORCE)
  message(STATUS "Build type defaulted to ${CMAKE_BUILD_TYPE}")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/firmware/third_party"
    CACHE PATH "Vendored third-party sources")

# Warnings applied to this project's own code only. Vendored sources and GoogleTest are
# never built with -Werror.
add_library(node_options INTERFACE)
target_compile_options(node_options INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
  -Wcast-qual -Wpointer-arith -Wundef -Wdouble-promotion
  $<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor -Woverloaded-virtual>)

add_library(node_options_strict INTERFACE)
target_link_libraries(node_options_strict INTERFACE node_options)
target_compile_options(node_options_strict INTERFACE -Werror)

# Target-tree flags, applied globally *before* any target exists.
#
# These must be directory-level rather than set on the executable: -mcpu and -mthumb
# are per-object properties, and setting them only on firmware.elf would leave
# sensor_core, the HAL and the FreeRTOS port compiled for the compiler's default
# architecture while the linker built a Cortex-M3 image — which links, and then faults
# on the first instruction.
#
# -ffunction-sections/-fdata-sections pair with --gc-sections at link time to drop the
# unused HAL modules, which is what keeps a 64 KB F103C8T6 comfortable.
if(CMAKE_CROSSCOMPILING)
  add_compile_options(
    -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -ffunction-sections -fdata-sections)
  add_link_options(-mcpu=cortex-m3 -mthumb -mfloat-abi=soft)
endif()

add_subdirectory(firmware)

if(CMAKE_CROSSCOMPILING)
  message(STATUS "Target build: ${CMAKE_SYSTEM_PROCESSOR}, image = firmware.elf")
else()
  enable_testing()
  add_subdirectory(tests)
endif()
