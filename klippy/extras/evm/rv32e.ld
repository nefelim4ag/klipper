PHDRS {
    text PT_LOAD;
}

SECTIONS {
    . = 0x0;
    .command : { KEEP(*(.command)) } :text
    .task : { KEEP(*(.task)) } :text
    .text   : { *(.text*) . = ALIGN(4); } :text
    .rodata : ALIGN(4) { *(.rodata*) *(.srodata*) . = ALIGN(4); } :text
    . = ALIGN(4);
    .data : { *(.data*) *(.sdata*) }
    .bss  : { *(.bss*) *(.sbss*) *(COMMON) }
    /DISCARD/ : {
        *(.eh_frame)
        *(.comment)
/*        *(.note.gnu.build-id)*/
/*        *(.note.ABI-tag)*/
/*        *(.interp)*/
/*        *(.gnu.hash)*/
/*        *(.dynsym)*/
/*        *(.dynstr)*/
/*        *(.dynamic)*/
/*        *(.got)*/
/*        *(.plt)*/
    }
}
