/* stm32f103c8t6_slot.ld — the linker script for the application AS INSTALLED IN A
 * SLOT: the artifact that host/flasher.py uploads (Phase 2).
 *
 * The bootloader runs images from the two slots at 0x08002000 / 0x08010000
 * (boot/include/boot/layout.h), and an image must be linked at its slot's base: the
 * vector table has to live there, because that is where the bootloader and, after the
 * FR-2.3 relocation, every interrupt look for it. ORIGIN and LENGTH are
 * BOOT_SLOT_A_BASE and BOOT_SLOT_SIZE from layout.h; a host test (test_boot_layout_ld)
 * cross-checks this file against that header so the two cannot drift, and LINK_DEPENDS
 * makes the link follow edits to this script.
 *
 * The companion build is firmware.elf with stm32f103c8t6.ld: the same application at
 * 0x08000000 for direct SWD flashing on a board with no bootloader and for bring-up.
 * Both scripts define _app_base = ORIGIN(FLASH), and the application sets SCB->VTOR
 * against it, so the same firmware is correct in either home.
 */
ENTRY(Reset_Handler)

_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
FLASH (rx)      : ORIGIN = 0x08002000, LENGTH = 56K
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Same derivation as the other two scripts: the top of RAM is one past its last byte. */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* This image's own home, used for the FR-2.3 vector-table relocation. */
_app_base = ORIGIN(FLASH);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}