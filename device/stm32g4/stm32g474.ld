ENTRY(Reset_Handler)

MEMORY
{
    /* -Wl,--defsym=CUSTOM_BOOTLOADER=1 links the application behind the 8K
       can bootloader, which jumps to 0x08002000. -Wl,--defsym=BOOTLOADER=1
       links that bootloader: the link fails if it outgrows its 8K. */
    FLASH (rx) : ORIGIN = DEFINED(CUSTOM_BOOTLOADER) ? 0x08002000 : 0x08000000,
                 LENGTH = DEFINED(BOOTLOADER) ? 8K : DEFINED(CUSTOM_BOOTLOADER) ? 504K : 512K
    RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 96K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
    } > FLASH

    .init_array :
    {
        __init_array_start = .;
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
    } > FLASH

    /* The first word of the ram is the flag that system_reset_to_bootloader
       (device.system) leaves for the can bootloader, across the reset, with
       this value in it. The bootloaders that are on boards look for that
       value at that address. */
    _bootloader_flag_value = 0xB007;
    .bootloader_flag (NOLOAD) :
    {
        _bootloader_flag = .;
        . = . + 4;
    } > RAM
    ASSERT(_bootloader_flag == 0x20000000, "the bootloader's flag has to stay the first word of the ram")

    /* Reset_Handler copies .data and zeroes .bss a word at a time */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT> FLASH
    _sidata = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    /* No exceptions or unwinding */
    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
}
