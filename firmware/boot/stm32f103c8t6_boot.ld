/* stm32f103c8t6_boot.ld — the bootloader's linker script.
 *
 * The bootloader is the 8 KB program at the base of flash, so FLASH here is exactly the
 * bootloader region from boot/include/boot/layout.h (NFR-2.9). Linker scripts cannot
 * include C headers, so this file carries the two numbers as literals and a host test
 * cross-checks them against layout.h — the same discipline as the application script.
 *
 * There is deliberately no heap and no C++ runtime in this image: freestanding C, one
 * stack, and everything static. The heap/stack reservation below exists only so the
 * linker errors if the static image would swallow the stack.
 */
ENTRY(Reset_Handler)

_Min_Stack_Size = 0x400;

MEMORY
{
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 8K
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Derived from the region, exactly like the application script after its _estack fix:
 * the top of RAM is one past its last byte, and a literal here invites the same
 * off-by-one that script shipped with. */
_estack = ORIGIN(RAM) + LENGTH(RAM);

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
    *(.rodata)         /* constants live in flash, including the CRC tables */
    *(.rodata*)
    . = ALIGN(4);
    _etext = .;
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
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } >RAM

  . = ALIGN(8);
  ._user_heap_stack :
  {
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Nothing links against libc/libm/libgcc by design; if something tries, drop it
   * loudly rather than let the image grow past 8 KB. */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}