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
  app/src/comms.cpp)

target_include_directories(sensor_core PUBLIC ${NODE_APP_INCLUDE_DIRS})
target_link_libraries(sensor_core PRIVATE node_options_strict)
target_compile_features(sensor_core PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# Target build: FreeRTOS port, STM32F103 HAL implementations, and the image.
# ---------------------------------------------------------------------------
