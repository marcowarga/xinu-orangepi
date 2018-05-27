/* meminit.c - meminit */

#include <xinu.h>

void	*minheap;	/* Start address of heap	*/
void	*maxheap;	/* End address of heap		*/
lid32	memlock;	/* Lock on low level memory manager */
uint32 get_stack_size(void); // get size of stack set up by start.S

/*------------------------------------------------------------------------
 * meminit - Initialize the free memory list for Orange Pi
 *------------------------------------------------------------------------
 */
void	meminit(void)
{
	struct	memblk *memptr;	/* Memory block pointer	*/

	/* Initialize the minheap and maxheap variables */

	minheap = (void *)&end;
	maxheap = (void *)(MAXADDR - get_stack_size());

	/* Initialize the memory list as one big block */

	memlist.mnext = (struct memblk *)minheap;
	memptr = memlist.mnext;

	memptr->mnext = (struct memblk *)NULL;
	memlist.mlength = memptr->mlength =
		(uint32)maxheap - (uint32)minheap;

	/* Initialize the low level memory manager spinlock */
	memlock = newlock();
}
