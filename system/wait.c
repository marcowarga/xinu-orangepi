/* wait.c - wait */

#include <xinu.h>

/*------------------------------------------------------------------------
 *  wait  -  Cause current process to wait on a semaphore
 *------------------------------------------------------------------------
 */
syscall	wait(
	  sid32		sem		/* Semaphore on which to wait  */
	)
{
	intmask mask;				/* Saved interrupt mask			*/
	struct	procent *prptr;		/* Ptr to process's table entry	*/
	struct	sentry *semptr;		/* Ptr to sempahore table entry	*/

	if (isbadsem(sem)) {
		return SYSERR;
	}
	semptr = &semtab[sem];

	mask = xsec_beg();
	lock(semptr->slock);

	if (semptr->sstate == S_FREE) {
		unlock(semptr->slock);
		xsec_end(mask);
		return SYSERR;
	}

	if (--(semptr->scount) < 0) {		/* If caller must block	*/
		prptr = &proctab[currpid];
		prptr->prstate = PR_WAIT;		/* Set state to waiting	*/
		prptr->prsem = sem;				/* Record semaphore ID	*/
		enqueue(currpid,semptr->squeue);/* Enqueue on semaphore	*/
		resched();						/*   and reschedule	*/
	}

	unlock(semptr->slock);
	xsec_end(mask);
	return OK;
}
