# ---------------------------------------------------------------------------
# Boot core (Phase 2) — target-independent pieces of the bootloader. Independent of
# the application core; declared first only so the tree reads as flash -> boot -> app.
# ---------------------------------------------------------------------------
add_subdirectory(boot)

# ---------------------------------------------------------------------------
# Application core — target-independent.
#
# Compiled identically in the host and target trees. It may include hal/ and rtos/
# headers (the interfaces) but must never include an STM32 device header or a FreeRTOS
# header; that is the boundary FR-0.6 and the Phase 3 port both depend on.
# ---------------------------------------------------------------------------
set(NODE_APP_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/app/include
  ${CMAKE_CURRENT_SOURCE_DIR}/hal/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rtos/include)

add_library(sensor_core STATIC
  app/src/i2c_recovery.cpp
  app/src/i2c_arbiter.cpp
  app/src/supervisor.cpp
  app/src/sensor.cpp
  app/src/acquisition.cpp
  app/src/control.cpp
  app/src/timing.cpp
  app/src/step_capture.cpp
  app/src/comms.cpp
  app/src/display.cpp
  app/src/node.cpp)

target_include_directories(sensor_core PUBLIC ${NODE_APP_INCLUDE_DIRS})
target_link_libraries(sensor_core PRIVATE node_options_strict)
target_compile_features(sensor_core PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# Target build: FreeRTOS port, STM32F103 HAL implementations, and the image.
# ---------------------------------------------------------------------------
if(CMAKE_CROSSCOMPILING)
  # -mcpu/-mthumb come from the top-level file, so they apply to every target in this
  # tree. Only link-time and target-specific pieces belong here.

  add_subdirectory(rtos/freertos)
  add_subdirectory(hal/stm32f1)

  add_executable(firmware.elf
    app/src/main_target.cpp
    # Startup and clock setup come from the vendored CMSIS device package. They are
    # listed as executable sources rather than hidden inside a static library: the
    # linker only pulls a library member that resolves an undefined reference, and the
    # vector table is not referenced by anything, so in a library it can be dropped and
    # the image links to a garbage reset vector.
    ${THIRD_PARTY_DIR}/cmsis-device-f1/Source/gcc/startup_stm32f103xb.s
    ${THIRD_PARTY_DIR}/cmsis-device-f1/Source/system_stm32f1xx.c)

  # Every object library is named explicitly, including the two internal ones
  # (*_kernel, *_vendor). CMake does not propagate an object library's objects through
  # another object library to the executable, so naming only the outer two here links an
  # image with every vendored symbol undefined — FreeRTOS's queue.c and the ST HAL's RCC
  # code simply do not arrive. Naming them is also self-documenting: this is what goes
  # into the image.
  target_link_libraries(firmware.elf PRIVATE
    sensor_core
    node_rtos_freertos
    node_rtos_freertos_kernel
    node_hal_stm32f1
    node_hal_stm32f1_vendor)
  # The entry point is this project's code, so it gets the full warning set too.
  target_link_libraries(firmware.elf PRIVATE node_options_strict)

  target_link_options(firmware.elf PRIVATE
    # Project-owned: the vendored script covers the whole xB density class and allows
    # 128K, which is more than the F103C8T6 in the BOM has. See the script's header.
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32f103c8t6.ld
    -Wl,--gc-sections
    -Wl,-Map=firmware.map
    -Wl,--print-memory-usage
    -nostartfiles
    -specs=nano.specs -specs=nosys.specs)

  # The script above is passed through target_link_options, which CMake does *not* treat as a
  # dependency of the link: editing the script left the executable stale until some unrelated
  # change forced a relink. That is exactly how a wrong _estack survived in this tree —
  # the fix was written, built, and silently not linked. LINK_DEPENDS puts the script in the
  # link rule's dependency list, so changing it relinks the image.
  set_target_properties(firmware.elf PROPERTIES
    LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/stm32f103c8t6.ld)

  # No SUFFIX is set: on a Generic system CMake appends none, and the target name
  # already carries the .elf, so setting one produced firmware.elf.elf.

  add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:firmware.elf> firmware.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware.elf> firmware.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:firmware.elf>
    COMMENT "Producing firmware.hex and firmware.bin")
endif()
