/* startup_boot.s — the bootloader's reset sequence.
 *
 * The smallest startup this project can have, and deliberately not the vendored CMSIS
 * one: that file initialises a C++ runtime with global constructors, which a
 * freestanding C bootloader must not carry. Ours does the two things a C program needs —
 * zero .bss, copy .data out of flash — and calls main. Every other exception or
 * interrupt vector points at Default_Handler, a dead spin: the bootloader uses none of
 * them, and if one fires anyway it is a fault worth freezing on, not a thing to paper
 * over by pretending it was handled.
 *
 * The vector table has 48 entries (medium-density F103): the two the hardware uses at
 * reset (initial MSP, reset PC) plus 46 more, all Default_Handler.
 */
    .syntax unified
    .cpu cortex-m3
    .thumb

    .section .isr_vector, "a", %progbits
    .word _estack
    .word Reset_Handler
    .rept 46
      .word Default_Handler
    .endr

    .section .text
    .thumb_func
    .global Reset_Handler
Reset_Handler:
    /* Zero .bss: every static in the bootloader starts at zero by the C rule. */
    ldr   r0, =_sbss
    ldr   r1, =_ebss
    movs  r2, #0
bss_loop:
    cmp   r0, r1
    bcs   bss_done
    str   r2, [r0]
    adds  r0, #4
    b     bss_loop
bss_done:
    /* Copy .data from its load address in flash to its runtime address in RAM. */
    ldr   r0, =_sdata
    ldr   r1, =_sidata
    ldr   r2, =_edata
data_loop:
    cmp   r0, r2
    bcs   data_done
    ldr   r3, [r1]
    str   r3, [r0]
    adds  r0, #4
    adds  r1, #4
    b     data_loop
data_done:
    bl    main
    /* main must not return; if it does, freeze where a debugger can find us. */
    b     .

    .thumb_func
    .global Default_Handler
Default_Handler:
    b     .
