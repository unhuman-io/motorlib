## Bootloader

Replaces built in bootloader. The built in bootloader should be disabled via option bytes.
It will always run at address 0x8000000. It will default to immediately transitioning to the application at 0x8001000 unless it is triggered in 3 different ways. 
1. Pin B8 set/pulled high.
2. IWDG or WWDG watchdog caused the reset
3. Software reset was called with the global ram variable go_to_bootloader set to 0xB007.

It 

The bootloader currently supports CAN and has a small subset of commands.
1. Erase pages
2. Read
3. Write
4. Exit

Each message received is acknowledged with either an ack or the requested data in the case of the read.

It communicates on a specific 7 bit can id, which is set in flash. If the bootloader sees a message from another device on the same bus with the same can id it will cease communication until reset. Presumably this should allow one to exit a state where multiple boards have been programmed with the same can id by allowing the first responder to take control and potentially have its can id (possibly temporarily) reprogrammed.
