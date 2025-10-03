MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
 . = ORIGIN(FLASH);
 .text : {
  *(.vectors)
  *(.text)
 } >FLASH

 . = ORIGIN(RAM);
 .data ALIGN(4) : {
  _data_start = .;
  *(.data)
  . = ALIGN(4);
  _data_end = .;
 } >RAM AT > FLASH

 .bss ALIGN(4) (NOLOAD) : {
  _bss_start = .;
  *(.bss)
  . = ALIGN(4);
  _bss_end = .;
 } >RAM

 _stacktop = ORIGIN(RAM) + LENGTH(RAM);
 _data_load = LOADADDR(.data);
}
