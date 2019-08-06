STARTUP(vectors.o)
ENTRY(reset_vector)
INPUT(extras.o)
GROUP(libtarget.a libgcc.a libsupc++.a)
MEMORY
{
    sram : ORIGIN = 0x20000000+0x000035E0,
    LENGTH = 0x00010000-0x000035E0 -4096
    flash : ORIGIN = 0x00044000, LENGTH = 0x0003C000
}
SECTIONS
{
    .debug_aranges 0 : { *(.debug_aranges) } .debug_pubnames 0 : { *(.debug_pubnames) } .debug_info 0 : { *(.debug_info) } .debug_abbrev 0 : { *(.debug_abbrev) } .debug_line 0 : { *(.debug_line) } .debug_frame 0 : { *(.debug_frame) } .debug_str 0 : { *(.debug_str) } .debug_loc 0 : { *(.debug_loc) } .debug_macinfo 0 : { *(.debug_macinfo) } .note.arm.ident 0 : { KEEP (*(.note.arm.ident)) } /DISCARD/ 0 : { *(.fini_array*) }
    .rom_vectors 0x00044000 : { __rom_vectors_vma = ABSOLUTE(.); . = .; KEEP (*(.vectors)) } > flash __rom_vectors_lma = LOADADDR(.rom_vectors);
    .rel.text : { *(.rel.text) *(.rel.text.*) *(.rel.gnu.linkonce.t*) } > flash .rela.text : { *(.rela.text) *(.rela.text.*) *(.rela.gnu.linkonce.t*) } > flash .rel.data : { *(.rel.data) *(.rel.data.*) *(.rel.gnu.linkonce.d*) } > flash .rela.data : { *(.rela.data) *(.rela.data.*) *(.rela.gnu.linkonce.d*) } > flash .rel.rodata : { *(.rel.rodata) *(.rel.rodata.*) *(.rel.gnu.linkonce.r*) } > flash .rela.rodata : { *(.rela.rodata) *(.rela.rodata.*) *(.rela.gnu.linkonce.r*) } > flash .rel.got : { *(.rel.got) } > flash .rela.got : { *(.rela.got) } > flash .rel.ctors : { *(.rel.ctors) } > flash .rela.ctors : { *(.rela.ctors) } > flash .rel.dtors : { *(.rel.dtors) } > flash .rela.dtors : { *(.rela.dtors) } > flash .rel.init : { *(.rel.init) } > flash .rela.init : { *(.rela.init) } > flash .rel.fini : { *(.rel.fini) } > flash .rela.fini : { *(.rela.fini) } > flash .rel.bss : { *(.rel.bss) } > flash .rela.bss : { *(.rela.bss) } > flash .rel.plt : { *(.rel.plt) } > flash .rela.plt : { *(.rela.plt) } > flash .rel.dyn : { *(.rel.dyn) } > flash
    .ARM.extab ALIGN (0x8) : { PROVIDE (__stext = ABSOLUTE(.));_stext = ABSOLUTE(.) ; . = .; *(.ARM.extab* .gnu.linkonce.armextab.*) } > flash . = ALIGN(8); __exidx_start = ABSOLUTE(.); .ARM.exidx ALIGN(8) : AT ((LOADADDR (.ARM.extab) + SIZEOF (.ARM.extab) + (8) - 1) & ~ ((8) - 1)) { *(.ARM.exidx* .gnu.linkonce.armexidx.*) . = .; } > flash __exidx_end = ABSOLUTE(.); .text ALIGN(8) : AT ((LOADADDR (.ARM.exidx) + SIZEOF (.ARM.exidx) + (8) - 1) & ~ ((8) - 1)) { *(.text*) *(.gnu.warning) *(.gnu.linkonce.t.*) *(.init) *(.glue_7) *(.glue_7t) __CTOR_LIST__ = ABSOLUTE (.); KEEP (*(SORT (.ctors*))) __CTOR_END__ = ABSOLUTE (.); __DTOR_LIST__ = ABSOLUTE (.); KEEP (*(SORT (.dtors*))) __DTOR_END__ = ABSOLUTE (.); } > flash _etext = .; PROVIDE (__etext = .);
    .fini ALIGN (0x8) : { . = .; *(.fini) } > flash
    .rodata ALIGN (0x8) : { . = .; *(.rodata*) *(.gnu.linkonce.r.*) } > flash
    .rodata1 ALIGN (0x8) : { . = .; *(.rodata1) } > flash
    .fixup ALIGN (0x8) : { . = .; *(.fixup) } > flash
    .gcc_except_table ALIGN (0x8) : { . = .; KEEP(*(.gcc_except_table)) *(.gcc_except_table.*) } > flash
    .got ALIGN (0x8) : { . = .; *(.got.plt) *(.got) _GOT1_START_ = ABSOLUTE (.); *(.got1) _GOT1_END_ = ABSOLUTE (.); _GOT2_START_ = ABSOLUTE (.); *(.got2) _GOT2_END_ = ABSOLUTE (.); } > flash
    .sram 0x20000300+0x000035E0 : AT ((LOADADDR (.got) + SIZEOF (.got) + (4) - 1) & ~ ((4) - 1)) { __sram_data_start = ABSOLUTE (.); *(.sram*) . = ALIGN (4); } > sram __srom_data_start = LOADADDR (.sram); __sram_data_end = .; PROVIDE (__sram_data_end = .); PROVIDE (__srom_data_end = LOADADDR (.sram) + SIZEOF(.sram));
    .data ALIGN (0x8) : AT ((LOADADDR (.sram) + SIZEOF (.sram) + (4) - 1) & ~ ((4) - 1)) { __ram_data_start = ABSOLUTE (.); *(.data*) *(.data1) *(.gnu.linkonce.d.*) . = ALIGN (4); KEEP(*( SORT (.ecos.table.*))) ; . = ALIGN (4); __init_array_start__ = ABSOLUTE (.); KEEP (*(SORT (.init_array.*))) KEEP (*(SORT (.init_array))) __init_array_end__ = ABSOLUTE (.); *(.dynamic) *(.sdata*) *(.gnu.linkonce.s.*) . = ALIGN (4); *(.2ram.*) } > sram __rom_data_start = LOADADDR (.data); __ram_data_end = .; PROVIDE (__ram_data_end = .); _edata = .; PROVIDE (edata = .); PROVIDE (__rom_data_end = LOADADDR (.data) + SIZEOF(.data));
    .fs_data ALIGN (0x8) : AT ((LOADADDR (.data) + SIZEOF (.data) + (4) - 1) & ~ ((4) - 1)) { __start_fs_data = ABSOLUTE (.); *(.fs_data*) __stop_fs_data = ABSOLUTE (.); } > sram __rom_start_fs_data = LOADADDR (.fs_data); PROVIDE (__rom_end_fs_data = LOADADDR (.fs_data) + SIZEOF(.fs_data));
    .bss ALIGN (0x8) : { __bss_start = ABSOLUTE (.); *(.scommon) *(.dynsbss) *(.sbss*) *(.gnu.linkonce.sb.*) *(.dynbss) *(.bss*) *(.gnu.linkonce.b.*) *(COMMON) __bss_end = ABSOLUTE (.); } > sram
    __heap1 = ALIGN (0x8);
    . = ALIGN(4); _end = .; PROVIDE (end = .);
}
hal_vsr_table = 0x20000000+0x000035E0;
hal_virtual_vector_table = hal_vsr_table + 128*4;
hal_startup_stack = 0x20000000 + 1024*64;
