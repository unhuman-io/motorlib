## Code documentation for startup

1. Check for iwdg watchdog reset -> go to bootloader
2. Check if software reset and go_to_bootloader is set to 0xB007 -> go to bootloader
3. Start iwdg
4. Copy .data and .ccmram sections from flash to ram
5. Zero fill uninitialized memory
6. Call SystemInit
    1. Enable FPU
    2. Point interrupt vector table to ccmram
7. Call board_init
    1. Call `board_rev`
    2. Call `SystemClock_Config()`
    3. Call pin config
8. Call libc_init_array -> static constructors
9. Go to main
    1. Call `system_init()`
    2. Repeatedly run `system_run()`
