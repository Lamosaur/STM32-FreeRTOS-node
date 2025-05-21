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
# Point STM32_TOOLCHAIN_DIR at any directory containing arm-none-eabi-gcc, or let the
# search below find one.
set(STM32_TOOLCHAIN_DIR "" CACHE PATH "Directory containing arm-none-eabi-gcc")

if(NOT STM32_TOOLCHAIN_DIR)
  file(GLOB _pio_toolchains
       "$ENV{HOME}/.platformio/packages/toolchain-gccarmnoneeabi*/bin")
  foreach(_dir ${_pio_toolchains} /usr/bin /usr/local/bin)
    if(EXISTS "${_dir}/arm-none-eabi-gcc")
      set(STM32_TOOLCHAIN_DIR "${_dir}")
      break()
    endif()
  endforeach()
endif()

if(NOT STM32_TOOLCHAIN_DIR)
  message(FATAL_ERROR
    "arm-none-eabi-gcc not found. Install a bare-metal ARM toolchain "
    "(e.g. 'pio pkg install -g -p toolchain-gccarmnoneeabi') or configure with "
    "-DSTM32_TOOLCHAIN_DIR=/path/to/bin")
endif()

message(STATUS "ARM toolchain: ${STM32_TOOLCHAIN_DIR}")

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
# They live in firmware/CMakeLists.txt (STM32_CPU_FLAGS) so the host build never sees
# them.
