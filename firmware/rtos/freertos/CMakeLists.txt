# ---------------------------------------------------------------------------
# node_rtos_freertos — the FreeRTOS kernel plus the Cortex-M3 port, bound to the
# rtos_iface_t the application layer is written against.
#
# Two targets rather than one, and both OBJECT libraries rather than STATIC, for two
# separate reasons that are both worth stating because both fail silently:
#
#   1. Vendored sources are compiled without this project's warning set (they are
#      third-party code and are never edited here), while rtos_freertos.c is this
#      project's code and is compiled with node_options_strict. That is why the sources
#      are split across node_rtos_freertos_kernel and node_rtos_freertos.
#
#   2. OBJECT libraries, not STATIC: the startup file declares SVC_Handler,
#      PendSV_Handler and SysTick_Handler as *weak* aliases, and the port takes those
#      slots by defining the same names. GNU ld does not extract an archive member to
#      satisfy a weak undefined reference, so with port.c inside a .a the image would
#      link, the vector table would keep the startup file's infinite-loop defaults, and
#      the scheduler would never start. An OBJECT library puts the objects in the link
#      unconditionally. --gc-sections still removes whatever is genuinely unreferenced.
# ---------------------------------------------------------------------------

set(FREERTOS_DIR ${THIRD_PARTY_DIR}/FreeRTOS-Kernel)

# The kernel sources are included from the vendored tree, and FreeRTOSConfig.h (this
# project's file, at firmware/FreeRTOSConfig.h) includes the CMSIS device header, so
# these paths are needed here as well as in the HAL. They are repeated rather than
# shared through a target because the HAL and the RTOS port are independent layers and
# hal/ is added to the build *after* this directory.
set(NODE_FREERTOS_CMSIS_INCLUDES
  ${THIRD_PARTY_DIR}/cmsis-core/Include
  ${THIRD_PARTY_DIR}/cmsis-device-f1/Include)

add_library(node_rtos_freertos_kernel OBJECT
  ${FREERTOS_DIR}/tasks.c
  ${FREERTOS_DIR}/queue.c
  ${FREERTOS_DIR}/list.c
  ${FREERTOS_DIR}/timers.c
  ${FREERTOS_DIR}/event_groups.c
  ${FREERTOS_DIR}/stream_buffer.c
  ${FREERTOS_DIR}/croutine.c
  ${FREERTOS_DIR}/portable/GCC/ARM_CM3/port.c
  ${FREERTOS_DIR}/portable/MemMang/heap_4.c)

# SYSTEM marks these as third-party headers, which keeps -Wundef/-Wconversion noise
# from inside the kernel headers out of this project's own diagnostics. The
# FreeRTOSConfig.h directory is deliberately *not* SYSTEM: it is ours and its warnings
# should be visible.
target_include_directories(node_rtos_freertos_kernel SYSTEM PUBLIC
  ${FREERTOS_DIR}/include
  ${FREERTOS_DIR}/portable/GCC/ARM_CM3
  ${NODE_FREERTOS_CMSIS_INCLUDES})
target_include_directories(node_rtos_freertos_kernel PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../..)   # firmware/FreeRTOSConfig.h
target_compile_definitions(node_rtos_freertos_kernel PUBLIC STM32F103xB)

# ---------------------------------------------------------------------------
# rtos_iface_t over FreeRTOS.
# ---------------------------------------------------------------------------
add_library(node_rtos_freertos OBJECT rtos_freertos.c freertos_hooks.c)
# PUBLIC so the target's entry point can include rtos_freertos.h to obtain the
# interface, without any other translation unit needing to know FreeRTOS exists.
target_include_directories(node_rtos_freertos PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../include)   # rtos/rtos.h, the frozen interface
target_link_libraries(node_rtos_freertos PUBLIC node_rtos_freertos_kernel)
target_link_libraries(node_rtos_freertos PRIVATE node_options_strict)
