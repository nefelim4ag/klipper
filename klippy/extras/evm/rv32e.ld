ENTRY(task)

PHDRS {
    text PT_LOAD;
}

SECTIONS {
    . = 0xffffffff; /* MUST be non zero to trigger PIE */
    .text : { *(.prog) *(.text*) } :text
    .data : { *(.data*) }
    .bss  : { *(.bss*)  }
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
