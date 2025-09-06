# CMake toolchain file for the STM32F103 (Cortex-M3) target build.
#
#   cmake -B build-arm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#
# The host build (no toolchain file) compiles the same target-independent application
# core and runs the unit tests; see the top-level CMakeLists.txt.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# We only ever compile and link a full image, never a bare executable, so let CMake
# skip its link-time compiler sanity check.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# --- Locate arm-none-eabi ------------------------------------------------
# This project is developed without root, so a system-wide install is not assumed.
# STM32_TOOLCHAIN_DIR may be set to any directory containing arm-none-eabi-gcc; otherwise
# one is chosen from the usual places.
#
# The choice is explicit and reported rather than left to a bare glob. This machine has
# two PlatformIO toolchains — GCC 12.3.1 and GCC 7.2.1 — and `file(GLOB)` returns them in
# whatever order the filesystem gives, so a first-match-wins search compiles the image
# with a compiler nobody chose. That is not hypothetical: it is how a build once came to
# be measured on 7.2.1 while the figure was reported as if it came from 12.3.1. The
# newest major version wins, the selection is forced into the cache so it can be read
# back, and the version is printed at configure time.
set(STM32_TOOLCHAIN_DIR "" CACHE PATH "Directory containing arm-none-eabi-gcc")
set(STM32_TOOLCHAIN_VERSION "" CACHE STRING "GCC version of the selected ARM toolchain")

function(_node_probe_toolchain dir out_version)
  if(EXISTS "${dir}/arm-none-eabi-gcc")
    execute_process(
      COMMAND "${dir}/arm-none-eabi-gcc" -dumpversion
      OUTPUT_VARIABLE _version OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET RESULT_VARIABLE _status)
    if(_status EQUAL 0 AND _version)
      set(${out_version} "${_version}" PARENT_SCOPE)
      return()
    endif()
  endif()
  set(${out_version} "" PARENT_SCOPE)
endfunction()

if(STM32_TOOLCHAIN_DIR)
  # Explicitly requested: honour it, and still report what it is.
  _node_probe_toolchain("${STM32_TOOLCHAIN_DIR}" STM32_TOOLCHAIN_VERSION)
  if(NOT STM32_TOOLCHAIN_VERSION)
    message(FATAL_ERROR
      "STM32_TOOLCHAIN_DIR='${STM32_TOOLCHAIN_DIR}' has no working arm-none-eabi-gcc")
  endif()
else()
  file(GLOB _node_pio_dirs
       "$ENV{HOME}/.platformio/packages/toolchain-gccarmnoneeabi*/bin")
  set(_node_best_dir "")
  set(_node_best_major -1)
  set(_node_best_version "")
  foreach(_dir ${_node_pio_dirs} /usr/bin /usr/local/bin)
    _node_probe_toolchain("${_dir}" _version)
    if(_version)
      # -dumpversion is "12.3.1" on current releases and a bare major on older ones, so
      # only the major is compared; the full string is what gets reported and recorded.
      string(REGEX REPLACE "\\..*$" "" _major "${_version}")
      if(_major GREATER _node_best_major)
        set(_node_best_major "${_major}")
        set(_node_best_dir "${_dir}")
        set(_node_best_version "${_version}")
      endif()
    endif()
  endforeach()

  if(NOT _node_best_dir)
    message(FATAL_ERROR
      "arm-none-eabi-gcc not found. Install a bare-metal ARM toolchain "
      "(e.g. 'pio pkg install -g -p toolchain-gccarmnoneeabi') or configure with "
      "-DSTM32_TOOLCHAIN_DIR=/path/to/bin")
  endif()

  set(STM32_TOOLCHAIN_DIR "${_node_best_dir}" CACHE PATH
      "Directory containing arm-none-eabi-gcc" FORCE)
  set(STM32_TOOLCHAIN_VERSION "${_node_best_version}" CACHE STRING
      "GCC version of the selected ARM toolchain" FORCE)
endif()

message(STATUS "ARM toolchain: ${STM32_TOOLCHAIN_DIR} (GCC ${STM32_TOOLCHAIN_VERSION})")

set(CMAKE_C_COMPILER   "${STM32_TOOLCHAIN_DIR}/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${STM32_TOOLCHAIN_DIR}/arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "${STM32_TOOLCHAIN_DIR}/arm-none-eabi-gcc")
set(CMAKE_OBJCOPY      "${STM32_TOOLCHAIN_DIR}/arm-none-eabi-objcopy")
set(CMAKE_OBJDUMP      "${STM32_TOOLCHAIN_DIR}/arm-none-eabi-objdump")
set(CMAKE_SIZE         "${STM32_TOOLCHAIN_DIR}/arm-none-eabi-size")
set(CMAKE_AR           "${STM32_TOOLCHAIN_DIR}/arm-none-eabi-ar")
set(CMAKE_RANLIB       "${STM32_TOOLCHAIN_DIR}/arm-none-eabi-ranlib")

set(CMAKE_FIND_ROOT_PATH "${STM32_TOOLCHAIN_DIR}/..")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# The F103 has no FPU: -mcpu and -mfloat-abi are fixed per board, not per build type.
# They live in CMakeLists.txt (STM32_CPU_FLAGS) so the host build never sees them.
