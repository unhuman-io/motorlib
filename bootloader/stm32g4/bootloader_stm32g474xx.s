
  .syntax unified
	.cpu cortex-m4
	.thumb

.global	g_pfnVectors
.global	Default_Handler


/* start address for the initialization values of the .data section.
defined in linker script */
.word	_sidata
/* start address for the .data section. defined in linker script */
.word	_sdata
/* end address for the .data section. defined in linker script */
.word	_edata
/* start address for the .bss section. defined in linker script */
.word	_sbss
/* end address for the .bss section. defined in linker script */
.word	_ebss



.equ  BootRAM,        0xF1E0F85F
/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

.set RCC_BASE,        0x40021000
.set RCC_CSR_OFFSET,  0x94
.set RCC_CSR_SFTRSTF_POS, 28
.set RCC_CSR_RMVF_POS,    23
.set RCC_CSR_IWDGRSTF_POS, 29
.set RCC_CSR_WWDGRSTF_POS, 30
.set RCC_CSR_PINRSTF_POS,    26

.set RCC_APB2SMENR,         0x40021080
.set RCC_SYSCFGEN_POS,    0
.set SYSCFG_MEMRMP,	      0x40010000

.set IWDG_KR,		0x40003000

    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
  	// start watchdog IWDG->KR = 0xCCCC;
#ifndef NO_WATCHDOG
	ldr	r0, =((1<<12) | (1<<11)) // debug pauses watchdogs
	ldr	r1, =0xE0042008
	str   r0, [r1]		// IWDG stop on debug
	ldr	r0, =0xCCCC
	ldr	r1, =IWDG_KR
	str	r0, [r1]		// start watchdog
	ldr	r0, =0x5555		// access IWDG_PR key
	str	r0, [r1]
	// wait for IWDR_SR PVU to be 0
pvu_zero_wait:
	ldr	r0, [r1, #12]
	cmp	r0, #1
	beq	pvu_zero_wait
	
	ldr	r0, =0
	str	r0, [r1, #4]	// Set PR to 0 (default but bootloader sets to 6)
#endif

	ldr r0, =RCC_BASE
	ldr r1, [r0, #RCC_CSR_OFFSET]
	// if (!(r1 & 0xFE000000)) a bootloader reset, don't erase previous flags
	tst r1, #(0xFE000000)
	//beq no_csr_copy
	ldr r2, =rcc_csr_copy
	str r1, [r2]

	orr r1, #(1<<RCC_CSR_RMVF_POS)
	str r1, [r0, #RCC_CSR_OFFSET]			// clear reset flags

	tst r1, #((1<<RCC_CSR_IWDGRSTF_POS) | (1<<RCC_CSR_WWDGRSTF_POS))		// watchdog reset
	bne run_bootloader
	tst r1, #(1<<RCC_CSR_SFTRSTF_POS)		// software reset
	beq run_app
	ldr r0, =go_to_bootloader
	ldr r1, [r0]
	ldr r2, =0xB007
	cmp r1, r2
	mov r1, #0
	str r1, [r0]
	beq run_bootloader

run_app:
    ldr     r0, =0x8002000 /* APP BASE */
    ldr     sp, [r0]     /* SP @ +0 */
    ldr     r0, [r0, #4]     /* PC @ +4 */
    bx      r0

run_bootloader:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b	LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit
  
/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_estack
  movs r3, #0x0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2], #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system intitialization function.*/
//    bl  SystemInit
/* Call static constructors */
    bl __libc_init_array
/* Call the application's entry point.*/
	bl	main

LoopForever:
    b LoopForever

.size	Reset_Handler, .-Reset_Handler
