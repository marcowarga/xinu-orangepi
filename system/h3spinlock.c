// Allwinner H3 hardware spinlock support
#include <h3spinlock.h>

#define BUS_CLK_GATING_REG1 ((volatile uint32_t*)(0x01c20000 + 0x0064))
#define BUS_SOFT_RST_REG1 ((volatile uint32_t*)(0x01c20000 + 0x02c4))
#define SPINLOCK_STATUS_REG0 ((volatile uint32_t*)(0x01C18000 + 0x0010))
#define SPINLOCK_LOCK_REG0 ((volatile uint32_t*)(0x01C18000 + 0x0100 + 0 * 0x4))

void h3_spinlock_init()
{
    *BUS_CLK_GATING_REG1 |= (1 << 22);
    *BUS_SOFT_RST_REG1 |= (1 << 22);
}

int h3_spinlock_tryacquire(uint32_t nr)
{
	volatile uint32_t* statreg = SPINLOCK_STATUS_REG0 + nr;
	volatile uint32_t* lockreg = SPINLOCK_LOCK_REG0 + nr;
	if (*statreg == 0)
	{
		if (*lockreg == 0)
		{
			return 1;
		}
	}
	return 0;
}

void h3_spinlock_acquire(uint32_t nr)
{
	while (!h3_spinlock_tryacquire(nr))
	{
	}
}

void h3_spinlock_release(uint32_t nr)
{
    volatile uint32_t* lockreg = SPINLOCK_LOCK_REG0 + nr;
    *lockreg = 0;
}
