OUTPUT_ARCH( "riscv" )

SECTIONS
{
  . = 0x00001000;
  .text.init : { *(.text.init) }
  .tohost ALIGN(0x1000) : { *(.tohost) }
  .text ALIGN(0x1000) : { *(.text) }
  .data ALIGN(0x1000) : { *(.data) }
  .bss : { *(.bss) }
  _end = .;
}

