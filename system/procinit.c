/* procinit.c - procinit */

#include <xinu.h>

char* null_stack[NCORE];	/* null process stack for each core */
lid32 proctablock;			/* lock on the process table */

/*------------------------------------------------------------------------
 *  procinit  - Initialize process variables
 *------------------------------------------------------------------------
 */
status procinit(void){

	uint32 i;					/* iterator over proctab */
	struct	procent	*prptr;		/* Ptr to process table entry	*/

	/* Initialize locks on the process table and global process count */

	proctablock = newlock();

	/* Count the Null processes as the first processes in the system */

	prcount = NCORE;

	/* Initialize process table entries free */

	for (i = 0; i < NPROC; i++) {
		prptr = &proctab[i];
		prptr->prstate = PR_FREE;
		prptr->prname[0] = NULLCH;
		prptr->prstkbase = NULL;
		prptr->prprio = 0;
		prptr->prlock = newlock();
		prptr->praff = CPU_NONE;
		prptr->prcpu = CPU_NONE;
	}

	/* Initialize the Null process entries */

	for(i = 0; i < NCORE; i++){
		prptr = &proctab[i];
		prptr->prstate = PR_CURR;
		prptr->prprio = 0;
		strncpy(prptr->prname, "prnullx", 8);
		prptr->prname[6] = i + 0x30; /* convert i to string and append */
		null_stack[i] = getstk(NULLSTK);
		prptr->prstkbase = null_stack[i];
		prptr->prstklen = NULLSTK;
		prptr->prstkptr = 0;
		cpidtab[i].cpid = i;
		prptr->praff = i;
		prptr->prcpu = i;
	}

	return OK;
}
