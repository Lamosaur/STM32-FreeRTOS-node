# ---------------------------------------------------------------------------
# Boot core — the target-independent pieces of Phase 2.
#
# Plain C on purpose: the bootloader is the smallest program in this project, and
# nothing in it needs a C++ runtime. The same library compiles into the host test
# suite (where gtest drives it from C++) and, later, into the bootloader image —
# that shared compilation is FR-0.6's habit applied to Phase 2.
# ---------------------------------------------------------------------------
add_library(boot_core STATIC
  src/crc.c
  src/metadata.c
  src/decision.c
  src/frame.c)

target_include_directories(boot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(boot_core PRIVATE node_options_strict)
target_compile_features(boot_core PUBLIC c_std_11)

# ---------------------------------------------------------------------------
# The bootloader image — a second, tiny freestanding program at 0x08000000.
# ---------------------------------------------------------------------------
if(CMAKE_CROSSCOMPILING)
  add_executable(bootloader.elf
    src/boot_main.c
    src/boot_uart.c
    src/flash.c
    startup_boot.s)

  # The device header (stm32f1xx.h) and core_cm3.h come from the vendored CMSIS
  # packages; nothing of the STM32 HAL is wanted here — the bootloader is deliberately
  # register-level (see the .c files). Boot core is plain C and is shared with the host
  # tests, which is how its bits stay verified.
  target_include_directories(bootloader.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${THIRD_PARTY_DIR}/cmsis-core/Include
    ${THIRD_PARTY_DIR}/cmsis-device-f1/Include)
  target_compile_definitions(bootloader.elf PRIVATE STM32F103xB)
  target_link_libraries(bootloader.elf PRIVATE boot_core node_options_strict)

  # The bootloader is freestanding C with no runtime: no newlib, no specs, no
  # startfiles (the project provides startup_boot.s). The custom script pins the
  # image to 8 KB, so exceeding the budget is a link error, which is the point.
  target_link_options(bootloader.elf PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32f103c8t6_boot.ld
    -nostdlib -nostartfiles
    -Wl,--gc-sections
    -Wl,-Map=bootloader.map
    -Wl,--print-memory-usage)

  add_custom_command(TARGET bootloader.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:bootloader.elf> bootloader.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:bootloader.elf>)
endif()