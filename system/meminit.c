/* meminit.c - meminit */

#include <xinu.h>

void	*minheap;	/* Start address of heap	*/
void	*maxheap;	/* End address of heap		*/
lid32	memlock;	/* Lock on low level memory manager */

/*------------------------------------------------------------------------
 * meminit - Initialize the free memory list for Orange Pi
 *------------------------------------------------------------------------
 */
void	meminit(void)
{
	struct	memblk *memptr;	/* Memory block pointer	*/

	/* Initialize the minheap and maxheap variables */

	minheap = (void *)&end;
    minheap += 16 * 0x100000; //mw reserve some space for core stacks until I've figured why MAXADDR doesn't work
	maxheap = (void *)MAXADDR;

	/* Initialize the memory list as one big block */

	memlist.mnext = (struct memblk *)minheap;
	memptr = memlist.mnext;

	memptr->mnext = (struct memblk *)NULL;
	memlist.mlength = memptr->mlength =
		(uint32)maxheap - (uint32)minheap;

	/* Initialize the low level memory manager spinlock */
	memlock = newlock();
}
