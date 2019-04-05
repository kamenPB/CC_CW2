/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

SECTIONS {
  /* assign load address (per  QEMU) */
  .       =     0x70010000;
  /* place text segment(s)           */
  .text : { kernel/lolevel.o(.text) *(.text .rodata) }
  /* place data segment(s)           */
  .data : {                         *(.data        ) }
  /* place bss  segment(s)           */
  .bss  : {                         *(.bss         ) }
  /* align       address (per AAPCS) */
  .       = ALIGN( 8 );
  /* allocate stack for irq mode     */
  .       = . + 0x00001000;
  tos_irq = .;
  /* allocate stack for svc mode     */
  .       = . + 0x00001000;
  tos_svc = .;

  .       = . + 0x00001000;
  tos_P1  = .;
  .       = . + 0x00001000;
  tos_P2  = .;
  .       = . + 0x00001000;
  tos_P3  = .;
  .       = . + 0x00001000;
  tos_P4  = .;
  .       = . + 0x00001000;
  tos_P5  = .;
  .       = . + 0x00001000;
  tos_P6  = .;
  .       = . + 0x00001000;
  tos_P7  = .;
  .       = . + 0x00001000;
  tos_P8  = .;
  .       = . + 0x00001000;
  tos_P9  = .;
  .       = . + 0x00001000;
  tos_P10  = .;
  .       = . + 0x00001000;
  tos_P11  = .;
  .       = . + 0x00001000;
  tos_P12  = .;
    .       = . + 0x00001000;
  tos_P13  = .;
    .       = . + 0x00001000;
  tos_P14  = .;
    .       = . + 0x00001000;
  tos_P15  = .;
    .       = . + 0x00001000;
  tos_P16  = .;
    .       = . + 0x00001000;
  tos_P17  = .;
    .       = . + 0x00001000;
  tos_P18  = .;
    .       = . + 0x00001000;
  tos_P19  = .;
    .       = . + 0x00001000;
  tos_P20  = .;

}
