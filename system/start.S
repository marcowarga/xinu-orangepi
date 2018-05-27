/* start.S - start, bzero */

#include <armv7a.h>

#define UART0LSR 0x01C28014
#define UART0RBR 0x01C28000
#define TX_READY 0x40

#define CPSR_MODE_USER			0x10
#define CPSR_MODE_FIQ			0x11
#define CPSR_MODE_IRQ			0x12
#define CPSR_MODE_SVR			0x13
#define CPSR_MODE_ABORT			0x17
#define CPSR_MODE_HYP			0x1A
#define CPSR_MODE_UNDEFINED		0x1B
#define CPSR_MODE_SYSTEM		0x1F
#define CPSR_IRQ_INHIBIT		0x80
#define CPSR_FIQ_INHIBIT		0x40

/*------------------------------------------------------------------------
 * start  -  Initial entry point for a Xinu image (ARM) for primary core
 *------------------------------------------------------------------------
 */
	.text
	.globl	start			/* Declare entry point global	*/

start:

	// routine taken from NetBSD
	// enter SVC mode and enable the SMP bit if necessary
	// note: if the SMP bit has changed, caches will be enabled
	// but the MMU will be disabled
	bl coretex_init

	// set up stacks for all modes of this core
	bl setup_stacks

	/* Use bzero (below) to zero out the BSS area */
	ldr	r0, =edata
	ldr	r1, =end
	bl	bzero

	/* Call nulluser to initialize the Xinu system  */
	/*    (Note: the call never returns)		*/
	bl	nulluser

/*------------------------------------------------------------------------
 * secondary_start  -  entry point for secondary cores
 *------------------------------------------------------------------------
 */
	.text
	.globl	secondary_start			/* Declare entry point global	*/

secondary_start:
	// NetBSD clears caches on secondary core startup
	// since we start up with secondary cores disabled,
	// this is likely not necessary though
	bl armv7_dcache_l1inv_all
	bl armv7_icache_inv_all

	// routine taken from NetBSD
	// enter SVC mode and enable the SMP bit if necessary
	// note: if the SMP bit has changed, caches will be enabled
	// but the MMU will be disabled
	bl coretex_init

	// disable unaligned access
	// (simply because its disabled on core0, too)
	bl disable_unaligned_access

	// set exception vector
	ldr r0, =exp_vector
	bl evec_set_addr

	// set page table adress, access control and enable the MMU
	ldr r0, =page_table
	bl mmu_set_ttbr
	ldr r0, =0xFFFFFFFF
	bl mmu_set_dacr
	isb
	dsb
	dmb
	bl mmu_enable

	// set up stacks for all modes of this core
	bl setup_stacks

	bl secondary_run	/* never returns */

/*------------------------------------------------------------------------
 * Set up stacks for all modes for the current core
 *------------------------------------------------------------------------
 */
#define STACK_ADDR        0x40200000
//#define STACK_ADDR        MAXADDR //mw causes crashes, have yet to figure out why
#define STACK_PER_MODE    0x8000
#define STACK_PER_CORE    STACK_PER_MODE * 6
#define SVR_STACK         STACK_PER_MODE
#define IRQ_STACK         STACK_PER_MODE * 1
#define FIQ_STACK         STACK_PER_MODE * 2
#define USER_STACK        STACK_PER_MODE * 3
#define ABORT_STACK       STACK_PER_MODE * 4
#define UNDEFINED_STACK   STACK_PER_MODE * 5

setup_stacks:

	// fetch core number into r11
	mrc p15, 0, r11, c0, c0, 5
	and r11, r11, #0x03

	// calc stack offset for this core
	ldr r1,=STACK_PER_CORE
    mul r0, r1, r11
	ldr r1,=STACK_ADDR
	sub r4, r1, r0

    // switch to interrupt mode, set sp
    mov r0, #(CPSR_MODE_IRQ | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
    msr cpsr_c, r0
    sub sp, r4, # IRQ_STACK

    // switch to fiq mode, set sp
    mov r0, #(CPSR_MODE_FIQ | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
    msr cpsr_c, r0
    sub sp, r4, # FIQ_STACK

    // switch to undefined exception mode, set sp
    mov r0, #(CPSR_MODE_UNDEFINED | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
    msr cpsr_c, r0
    sub sp, r4, # UNDEFINED_STACK

    // switch to data abort exception mode, set sp
    mov r0, #(CPSR_MODE_ABORT | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
    msr cpsr_c, r0
    sub sp, r4, # ABORT_STACK

    // switch to user mode, set sp
    mov r0, #(CPSR_MODE_SYSTEM | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
    msr cpsr_c, r0
    sub sp, r4, # USER_STACK

    // switch to SVC mode, set sp
    mov r0, #(CPSR_MODE_SVR | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
    msr cpsr_c, r0
    sub sp, r4, # SVR_STACK

	bx lr

/*------------------------------------------------------------------------
 * Function to zero memory (r0 is lowest addr; r1 is highest)
 *------------------------------------------------------------------------
 */
bzero:
	mov	r2, #0					/* Round address to multiple	*/
	add	r0, r0, #3				/*   of four by adding 3 and	*/
	and	r0, r0, #0xFFFFFFFC		/*   taking the result module 4	*/
1:	cmp	r0, r1					/* Loop until last address	*/
	bhs	2f						/*   has been reached		*/
	str	r2, [r0]				/* Zero four-byte word of memory*/
	add	r0, r0, #4				/* Move to next word		*/
	b	1b						/* Continue to iterate		*/
2:	mov	pc, lr					/* Return to caller		*/

/* The cycle count register is a 32 bit register
 * This is for a Cortex A8 arm chip.
 */
	.globl get_ccnt
get_ccnt:
	mrc     p15, 0, r0, c9, c13, 0
	bx	lr

	.globl set_ccnt
set_ccnt:
	mcr     p15, 0, r0, c9, c13, 0
	bx	lr

/* The performance monitor control register controls
 * the behavior of the CCNT register.
 */
	.globl get_pmcr
get_pmcr:
	mrc     p15, 0, r0, c9, c12, 0
	bx	lr

	.globl set_pmcr
set_pmcr:
	mcr     p15, 0, r0, c9, c12, 0
	bx	lr

/* Counter enable register.
 */
	.globl get_cena
get_cena:
	mrc     p15, 0, r0, c9, c12, 1
	bx	lr

	.globl set_cena
set_cena:
	mcr     p15, 0, r0, c9, c12, 1
	bx	lr

/* Counter disable register.
 */
	.globl get_cdis
get_cdis:
	mrc     p15, 0, r0, c9, c12, 2
	bx	lr

/* Read access to various registers
 */
	.globl get_sctlr
get_sctlr:
	mrc     p15, 0, r0, c1, c0, 0
	bx	lr

	.globl get_actlr
get_actlr:
	mrc     p15, 0, r0, c1, c0, 1
	bx	lr

	.globl get_ttbr0
get_ttbr0:
	mrc     p15, 0, r0, c2, c0, 0
	bx	lr
	.globl get_ttbcr
get_ttbcr:
	mrc     p15, 0, r0, c2, c0, 2
	bx	lr

/* disable unaligned access
 */
disable_unaligned_access:
    mrc      p15, 0, r0, c1, c0, 0   // read SCTLR
    orr      r0, r0, #2              // A (no unaligned access fault)
    mcr      p15, 0, r0, c1, c0, 0   // write SCTLR
    bx lr

/*------------------------------------------------------------------------
 * Page Table
 *------------------------------------------------------------------------
 */
.data
/* Page table base address must be aligned at 16 KB boundary */
	.align	14
	.globl page_table
page_table:
	.space	(0x1000 * 4)

/*------------------------------------------------------------------------
 * bputc	-	simple polled output, prints C to screen for assembly
 				debugging when kputc won't work
 *------------------------------------------------------------------------
 */
.globl bputc
bputc:
	ldr r0, =UART0LSR		/* point to uart line status reg */
	ldr r1, [r0]			/* get value of line status reg */
	and r1, r1, #TX_READY	/* mask tx ready bit bit */
    cmp r1, #TX_READY		/* check for bit set */
	bne bputc				/* keep poling until bit is set */
	ldr r0, =UART0RBR		/* point to TX register */
	mov r1, #67				/* decimal 67 is capitol C */
	str r1, [r0]
	mov pc, lr

