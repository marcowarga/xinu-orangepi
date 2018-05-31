/* paging.c - paging_init */

#include <xinu.h>

/*------------------------------------------------------------------------
 * paging_init  -  Initialize Page Tables
 *------------------------------------------------------------------------
 */
status	paging_init (void) {

	int32	i;		/* Loop counter			*/

	// NOTE: U-boot starts us up in secure mode, so we
	//       have to create according secure mode descriptors
	//       without the NS flag beeing set.

	/* First, initialize PDEs for device memory	*/
	/* AP[2:0] = 0b011 for full access		*/
	/* TEX[2:0] = 0b000 and B = 1, C = 0 for	*/
	/*		shareable dev. mem.		*/
	/* S = 1 for shareable				*/
	/* NS = 0 for secure mode			*/

	for(i = 0; i < 1024; i++) { //TODO changed from 2048

		page_table[i] = ( PDE_PTSEC	|
						PDE_B		|
						PDE_AP3		|
						PDE_TEX0	|
						PDE_S		|
						PDE_XN		|
						(i << 20) );
	}

	/* Now, initialize normal memory		*/
	/* AP[2:0] = 0b011 for full access		*/
	/* TEX[2:0] = 0b101 B = 1, C = 1 for write back	*/
	/*	write alloc. outer & inner caches	*/
	/* S = 1 for shareable				*/
	/* NS = 0 for secure mode mem.		*/

	for(i = 1024; i < 4096; i++) {

		page_table[i] = ( PDE_PTSEC	|
						PDE_B		|
						PDE_C		|
						PDE_AP3		|
						PDE_TEX5	|
//						PDE_S		|	// this will make ldrex/strex data abort
						(i << 20) );
	}

	/* Set the Translation Table Base Address Register */
    asm volatile ("isb\ndsb\ndmb\n");

	return OK;
}
