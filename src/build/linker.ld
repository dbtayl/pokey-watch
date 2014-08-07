/******************************************************************************
 * Copyright (c) 2011, Universal Air Ltd. All rights reserved.
 * Source and binaries are released under the BSD 3-Clause license
 * See readme_forebrain.txt files for the text of the license
 ******************************************************************************/

MEMORY
{
  flash(rx): ORIGIN = 0x00000000, LENGTH = 32K
  sram(rwx): ORIGIN = 0x10000000+384, LENGTH = 8K-384
}

__top_flash = 0x0 + 0x8000;
__top_sram = 0x10000000 + 0x2000;
  
ENTRY(Reset_Handler)

SECTIONS
{
	.text :
	{
	KEEP(*(.vectors))
    . = 0x000002FC;
    KEEP(*(.crp))
	*(.text*)
	*(.rodata*)
	} > flash

	.ARM.extab : 
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > flash

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > flash
	__exidx_end = .;

	_etext = .;

	.data : AT (__exidx_end)
	{
		__data = .;
		*(vtable)
		*(.data*)
		__edata = .;
	} > sram

	/* zero initialized data */
	.bss :
	{
		__bss = .;
		*(.bss*)
		*(COMMON)
		__ebss = .;
	} > sram

	end = .;

	/* GDB workaround */
	__stack_top = __top_sram - 16;
}
