# Vendored third-party sources

These are pinned, pruned copies of upstream sources, committed into the repository so
that firmware builds are reproducible and work offline (no configure-time network
access). Nothing here is modified in substance; only whole files and directories that
this project does not compile have been deleted, and every deletion is listed below.

Do not edit files in this directory. To update a component, re-clone at a new tag,
re-apply the pruning described here, and record the new commit in this file.

## Components

| Directory | Upstream | Tag | Commit | Licence |
|-----------|----------|-----|--------|---------|
| `FreeRTOS-Kernel/` | [FreeRTOS/FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel) | `V11.3.0` | `81896bd1a2d883a362ad21e5af93c4cb040f6f8c` | MIT |
| `cmsis-core/` | [STMicroelectronics/STM32CubeF1](https://github.com/STMicroelectronics/STM32CubeF1) `Drivers/CMSIS/Core/Include` | `v1.8.7` | `d12e75247d5bcedc734f829b394517ab4c2726e3` | Apache-2.0 |
| `cmsis-device-f1/` | [STMicroelectronics/cmsis_device_f1](https://github.com/STMicroelectronics/cmsis_device_f1) | `v4.3.5` | `8a76309ed1250d817e9c888c4417171d2ba3ba63` | Apache-2.0 |
| `stm32f1xx-hal-driver/` | [STMicroelectronics/stm32f1xx_hal_driver](https://github.com/STMicroelectronics/stm32f1xx_hal_driver) | `v1.1.10` | `77fbb30b7a1d02533980400083e48c559aae5a4f` | BSD-3-Clause |

`cmsis-core/` comes from the CubeF1 release rather than from
[ARM-software/CMSIS_5](https://github.com/ARM-software/CMSIS_5) directly, because that
is the CMSIS Core revision the CubeF1 v1.8.7 device headers are tested against.
CubeF1 v1.8.7 itself is not vendored: it is a ~200 MB superproject that carries every
STM32F1 part plus middleware, and its `Drivers/CMSIS/Device` and `Drivers/STM32F1xx_HAL_Driver`
entries are git submodules pointing at the two repositories pinned above.

## Pruning applied

### `FreeRTOS-Kernel/`

Vendored at **V11.3.0**, deliberately rather than the later V11.3.1: the kernel version is
pinned to the release this project targets, and a patch-level bump is taken as a
considered change rather than something that arrives with a re-clone. The commit above is
the annotated tag object; the tree it points at is
`9b777ae5c5b8e9e456065a00294d1e5f5f9facf5`.

- Removed `.git/`, `.github/` (CI workflows, issue templates, scripts), `.gitattributes`,
  `.git-blame-ignore-revs`, `.gitmodules`, `examples/`, the `*.spdx*` SBOM files,
  `cspell.config.yaml`, `manifest.yml`, `MISRA.md`, and the upstream `.url` shortcuts.
  `.gitmodules` goes because the ports it references are not vendored here; only the
  Cortex-M3 GCC port is.
- Kept only the Cortex-M3 GCC port in `portable/`: `portable/GCC/ARM_CM3/`. Every other
  architecture port is deleted; the target is an STM32F103 (Cortex-M3).
- Kept only `portable/MemMang/heap_4.c`. `heap_4` is the intended allocator: it
  coalesces adjacent free blocks, so a long unattended run (NFR-0.7) cannot fragment the
  heap into exhaustion.
- The kernel and CMakeLists at the root are kept as upstream ships them, but
  `firmware/rtos/freertos/CMakeLists.txt` states its own source list rather than
  including upstream's: the upstream file expects the surrounding project layout and the
  ports it would pull in are not here.

### `cmsis-core/`

- Kept only `Include/`. The `armcc` and `iccarm` compiler-specific headers were removed;
  this project builds with GCC only (`cmsis_gcc.h`, `cmsis_compiler.h` are retained).

### `cmsis-device-f1/`

- `Include/` reduced from all 15 STM32F1 device headers to the ones this project can
  target: `stm32f1xx.h` (the dispatcher), `stm32f103xb.h` (medium-density F103, which
  covers the STM32F103C8T6 used in Phases 0-2), `stm32f103x6.h` (low-density, kept so
  the board can drop to a smaller part without re-vendoring), and `system_stm32f1xx.h`.
  Removed: `stm32f100x*`, `stm32f101x*`, `stm32f102x*`, `stm32f103xe/xg`,
  `stm32f105xc`, `stm32f107xc`.
- `Source/Templates/arm/` and `Source/Templates/iar/` removed (vendor toolchain startup
  files); `Source/Templates/gcc/` retained.
- Of the GCC startup files, only `startup_stm32f103xb.s` is kept, matching the device
  header above.
- Of `Source/Templates/gcc/linker/`, only `STM32F103XB_FLASH.ld` is kept.

### `stm32f1xx-hal-driver/`

- Removed `.git/`, `.github/`, `_htmresc/`, and the upstream `CMakeLists.txt`.
- `Inc/` and `Src/` are kept complete (62 sources, 68 headers). The driver is not
  pruned because unused translation units cost nothing at link time under
  `-ffunction-sections`/`--gc-sections`, and a missing source is a confusing failure
  later. `firmware/CMakeLists.txt` compiles an explicit subset.
