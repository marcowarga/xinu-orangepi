/* mmu.c - mmu_enable, mmu_disable, mmu_set_ttbr */

#include <xinu.h>


/*------------------------------------------------------------------------
 * mmuinit  -  Initialize the MMU
 *------------------------------------------------------------------------
 */
void mmuinit(void){
		/* Disable all caches */

		cache_disable_all();

		/* Invalidate all caches */

		cache_inv_all();

		/* Invalidate the TLB */

		tlb_inv_all();

		/* Make sure MMU is disabled */

		mmu_disable();

		/* Initialize page tables */

		paging_init();

		/* Make sure all memory operations are completed */

		asm volatile (
				"dsb\n"
				"dmb\n"
		);

		/* Enable caches  */

		cache_enable_all();

		/* Turn on the MMU */

		mmu_enable();
}

/*------------------------------------------------------------------------
 * mmu_enable  -  Enable the MMU
 *------------------------------------------------------------------------
 */
void	mmu_enable (void) {

	asm volatile (
			/* Read the Control Register */

			"mrc	p15, 0, r0, c1, c0, 0\n"

			/* Set the MMU Enable bit */

			"orr	r0, #0x00000001\n"

			// TODO: move this stuff to start.S?
//			/* Clear the TRE bit */
//
//			"bic	r0, #0x10000000\n"
//
//			/* Clear the AFE bit */
//
//			"bic	r0, #0x20000000\n"
//
//			/* Clear the EE bit */
//
//			"bic	r0, #0x02000000\n"
//
//			/* Clear the TE bit */
//
//			"bic	r0, #0x40000000\n"

			/* Write the new Control Register */

			"mcr	p15, 0, r0, c1, c0, 0\n"
			"isb\n"

			:	/* Output	*/
			:	/* Input	*/
			: "r0"	/* Clobber	*/
		     );
}

/*------------------------------------------------------------------------
 * mmu_disable  -  Disable the MMU
 *------------------------------------------------------------------------
 */
void	mmu_disable (void) {
	asm volatile (
			/* Read the Control Register */

			"mrc	p15, 0, r0, c1, c0, 0\n"

			/* Reset the MMU Enable bit */

			"bic	r0, #0x00000001\n"

			/* Reset the C bit */

			"bic	r0, #0x00000004\n"

			/* Write the new Control Register */

			"mcr	p15, 0, r0, c1, c0, 0\n"

			"isb\n"
			"dmb\n"
			"dsb\n"

			/* Invalidate branch predictor array */

			"mov	r0, #0\n"
			"mcr	p15, 0, r0, c7, c5, 6\n"
			"isb\n"

			:	/* Output	*/
			:	/* Input	*/
			: "r0"	/* Clobber	*/
		     );
}

/*------------------------------------------------------------------------
 * mmu_set_ttbr  -  Set the Translation Table Base Register
 *------------------------------------------------------------------------
 */
void	mmu_set_ttbr (
		void	*ttbaddr	/* Translation Tab. Base Address*/
		)
{
	asm volatile (

			/* We want to use TTBR0 only, 16KB page table */

			"ldr	r0, =0x00000000\n"

			/* Write the value into TTBCR */

			"mcr	p15, 0, r0, c2, c0, 2\n"

			/* Load the base address in r0 */

			"mov	r0, %0\n"

			/* Make the translation table walk cacheable */

			"orr	r0, #0x00000001\n"

			/* Write the new TTBR0 */

			"mcr	p15, 0, r0, c2, c0, 0\n"

			/* Write the new TTBR1 */

			"mcr	p15, 0, r0, c2, c0, 1\n"

			/* Perform memory synchronization */
			"isb\n"
			"dsb\n"
			"dmb\n"

			:		/* Output	*/
			: "r" (ttbaddr)	/* Input	*/
			: "r0"		/* Clobber	*/
		);
}
