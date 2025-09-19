# ---------------------------------------------------------------------------
# node_hal_stm32f1 — the STM32F103C8T6 implementation of the hal/*.h interfaces.
#
# Two targets, mirroring the RTOS directory and for the same reason: the vendored ST HAL
# driver is third-party code and is never built with this project's -Werror warning set,
# while hal_*_stm32f1.c is this project's code and is.
#
# They are OBJECT libraries rather than STATIC ones because several symbols here are
# *strong overrides of weak defaults* inside the vendored driver (HAL_InitTick,
# HAL_MspInit, HAL_I2C_MspInit/MspDeInit, HAL_UART_MspInit/MspDeInit,
# HAL_ADC_MspInit, HAL_ADC_ConvHalfCpltCallback, HAL_ADC_ConvCpltCallback,
# HAL_ADC_ErrorCallback, HAL_I2C_MasterTxCpltCallback, HAL_I2C_ErrorCallback,
# DMA1_Channel1_IRQHandler). A weak definition inside an
# archive member wins by default, so a static library would silently keep the stock
# SysTick-based HAL_InitTick — which would fight FreeRTOS for SysTick. As objects, both
# definitions are always present and the linker resolves the strong one. --gc-sections
# still discards everything genuinely unreferenced.
# ---------------------------------------------------------------------------

set(HAL_DIR ${THIRD_PARTY_DIR}/stm32f1xx-hal-driver)

# Only the modules this firmware calls into. The ST driver is complete in the vendored
# tree, so the list is explicit rather than a glob: adding a module here is a deliberate
# statement that the firmware uses it.
add_library(node_hal_stm32f1_vendor OBJECT
  ${HAL_DIR}/Src/stm32f1xx_hal.c
  ${HAL_DIR}/Src/stm32f1xx_hal_adc.c
  ${HAL_DIR}/Src/stm32f1xx_hal_adc_ex.c
  ${HAL_DIR}/Src/stm32f1xx_hal_cortex.c
  ${HAL_DIR}/Src/stm32f1xx_hal_dma.c
  ${HAL_DIR}/Src/stm32f1xx_hal_gpio.c
  ${HAL_DIR}/Src/stm32f1xx_hal_i2c.c
  ${HAL_DIR}/Src/stm32f1xx_hal_iwdg.c
  ${HAL_DIR}/Src/stm32f1xx_hal_pwr.c
  ${HAL_DIR}/Src/stm32f1xx_hal_rcc.c
  ${HAL_DIR}/Src/stm32f1xx_hal_rcc_ex.c
  ${HAL_DIR}/Src/stm32f1xx_hal_tim.c
  ${HAL_DIR}/Src/stm32f1xx_hal_tim_ex.c
  ${HAL_DIR}/Src/stm32f1xx_hal_uart.c)

# SYSTEM: the vendored HAL and CMSIS headers are third-party, so their internal warnings
# are not this project's to fix. USE_HAL_DRIVER is what makes stm32f1xx.h pull in
# stm32f1xx_hal.h (and through it HSE_VALUE, which system_stm32f1xx.c needs even though
# it is not HAL code). Both are PUBLIC because the executable's own sources —
# main_target.cpp and system_stm32f1xx.c — include the device header too.
target_include_directories(node_hal_stm32f1_vendor SYSTEM PUBLIC
  ${HAL_DIR}/Inc
  ${THIRD_PARTY_DIR}/cmsis-core/Include
  ${THIRD_PARTY_DIR}/cmsis-device-f1/Include)
target_compile_definitions(node_hal_stm32f1_vendor PUBLIC
  USE_HAL_DRIVER
  STM32F103xB)

# The vendored driver includes stm32f1xx_hal_conf.h, which is this project's file and
# lives here, so the vendor target needs this directory as well. Deliberately not SYSTEM:
# the configuration header is ours, and its warnings should be visible.
target_include_directories(node_hal_stm32f1_vendor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ---------------------------------------------------------------------------
# The implementation itself. The include directory is deliberately NOT SYSTEM: this is
# where stm32f1xx_hal_conf.h and hal_stm32f1.h live, and warnings in them are ours.
# ---------------------------------------------------------------------------
add_library(node_hal_stm32f1 OBJECT
  hal_stm32f1.c
  hal_adc_stm32f1.c
  hal_actuator_stm32f1.c
  hal_gpio_stm32f1.c
  hal_i2c_stm32f1.c
  hal_uart_stm32f1.c
  hal_watchdog_stm32f1.c
  hal_system_stm32f1.c)

target_include_directories(node_hal_stm32f1 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}                 # stm32f1xx_hal_conf.h, hal_stm32f1.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../include)     # the hal/ interfaces it implements
target_link_libraries(node_hal_stm32f1 PUBLIC node_hal_stm32f1_vendor)

# The I2C DMA path and the UART receive path use FreeRTOS semaphores, so the HAL depends
# on the RTOS port. That direction is deliberate: hal/i2c.h's transfer_wait() contract is
# "wait for the transfer", and blocking on a semaphore is the only implementation that
# leaves the CPU free — which is the entire point of FR-0.5. This is also why
# rtos/freertos is added to the build before hal/stm32f1.
target_link_libraries(node_hal_stm32f1 PUBLIC node_rtos_freertos)

target_link_libraries(node_hal_stm32f1 PRIVATE node_options_strict)
