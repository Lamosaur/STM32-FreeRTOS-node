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



# ---------------------------------------------------------------------------
# Target build: FreeRTOS port, STM32F103 HAL implementations, and the image.
# ---------------------------------------------------------------------------
