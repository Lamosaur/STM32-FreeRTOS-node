/* stm32f103c8t6.ld — the linker script for this project's target part.
 *
 * This is the vendored CMSIS device package's STM32F103XB_FLASH.ld (cmsis_device_f1
 * v4.3.5) with one substantive change: FLASH is 64K, not 128K. The vendored script
 * covers the whole STM32F103xB density class, which spans both the 64 KB C8 and the
 * 128 KB CB, so taking it unmodified lets the linker accept an image that does not fit
 * the part on the board — turning a build-time error into a board that boots to nothing.
 *
 * Many "Blue Pill" F103C8 boards are in fact fitted with 128 KB of flash silicon and
 * report it. Relying on that is not portable and is not done here: the image is kept
 * within the datasheet limit of the part named in the schematic and the BOM.
 *
 * Everything below this header is the vendored file verbatim, including its own licence
 * header, so the provenance stays visible.
 */
/*
*****************************************************************************
**
**  File        : STM32F103XB_FLASH.ld
**
**  Abstract    : Linker script for STM32F103xB Device with
**                128KByte FLASH, 20KByte RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
**  (c)Copyright Ac6.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Ac6 permit registered System Workbench for MCU users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the System Workbench for MCU toolchain.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 64K
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Highest address of the user mode stack: one past the last byte of RAM, because the stack
 * is full-descending and the first push writes *below* this address.
 *
 * Derived from the MEMORY block above rather than written out, because the value this file
 * inherited from ST's own F103xB script was wrong and nothing caught it. That script pairs
 *
 *   _estack = 0x20004FFF;           (its comment reads "end of RAM")
 *
 * with
 *
 *   RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
 *
 * and 0x20000000 + 20K is 0x20005000, not 0x20004FFF. The inherited value is the address of
 * the last *byte* of RAM, which leaves the initial MSP odd: not even word-aligned, and
 * ARMv7-M requires word alignment for the load/store-multiple instructions every function
 * prologue uses. Whether this particular die tolerates that is not worth discovering on a
 * device that has to run unattended for 72 h (NFR-0.7) — and an odd initial stack pointer
 * is wrong on the ABI's terms regardless of what one chip does with it. Expressing the value
 * as an expression over the region makes the two impossible to disagree.
 *
 * This does not move any code or data: it changes the stack's top by one byte, and the
 * reserved-heap-and-stack section below is sized by _Min_Heap_Size and _Min_Stack_Size
 * rather than by this address.
 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* This image's own home, used for the FR-2.3 vector-table relocation: main_target.cpp
 * sets SCB->VTOR against it, so this base build and the slot build name the same
 * symbol and both place the vector table correctly for wherever they were linked. */
_app_base = ORIGIN(FLASH);

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
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

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
