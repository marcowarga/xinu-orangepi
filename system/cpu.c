/*  cpu.c */
#include <xinu.h>
#include <h3spinlock.h>

#define dsb() asm volatile ("dsb");

extern uint32 exp_vector[];

uint32 get_sctlr(void);
uint32 get_actlr(void);
uint32 get_scr(void);
uint32 get_ttbr0(void);
uint32 get_ttbcr(void);
uint32 get_prrr(void);
uint32 get_nmrr(void);

void led_init(void);
void pwr_on(void);
void pwr_off(void);
void status_on(void);
void status_off(void);
void gpio_blink_red(void);
void gpio_blink_green(void);

uint32 get_ccnt(void);
uint32 get_pmcr(void);
void perfcounters_init()
{
	asm volatile ("MCR p15, 0, %0, c9, c12, 0\t\n" :: "r"(0x00000007));
	asm volatile ("MCR p15, 0, %0, c9, c12, 1\t\n" :: "r"(0x8000000f));
	asm volatile ("MCR p15, 0, %0, c9, c12, 3\t\n" :: "r"(0x8000000f));
}

#define CCU_REG_BASE 0x01c20000
#define PLL_CPUX_CTRL_REG ((volatile uint32*)(CCU_REG_BASE + 0x0000))
#define CPUX_AX1_CFG_REG ((volatile uint32*)(CCU_REG_BASE + 0x0050))
uint32 get_cpuclk(void)
{
	uint64 freq = 0;
	uint32 cpux_cfg = *CPUX_AX1_CFG_REG;
	uint32 clksrc = (cpux_cfg & (3 << 16)) >> 16;

	switch(clksrc)
	{
		case 0: // LOSC
		{
			freq = 32768;
			break;
		}
		case 1: // OSC24M
		{
			freq = 24000000;
			break;
		}
		case 2: // PLLCPUX
		case 3: // PLLCPUX
		{
			uint32 pll_cpux_ctrl = *PLL_CPUX_CTRL_REG;
			uint32 n = ((pll_cpux_ctrl & (31 << 8)) >> 8) + 1;
			uint32 k = ((pll_cpux_ctrl & (3 << 4)) >> 4) + 1;
			uint32 m = ((pll_cpux_ctrl & (3 << 0)) >> 0) + 1;
			freq = ((uint64)24000000 * n * k / m);
			break;
		}
	}

	return freq & 0xffffffff;
}

#define CPUCFG_REG_BASE 0x01f01c00
#define CPUCFG_CNT64_CTRL_REG ((volatile uint32*)(CPUCFG_REG_BASE + 0x0280))
#define CPUCFG_CNT64_LOW_REG ((volatile uint32*)(CPUCFG_REG_BASE + 0x0284))
#define CPUCFG_CNT64_HIGH_REG ((volatile uint32*)(CPUCFG_REG_BASE + 0x0288))
uint64 get_cnt64()
{
	uint64 cnt;

	//mutex_enter(&sc->sc_lock);
	// set read latch
	*CPUCFG_CNT64_CTRL_REG |= (1 << 1);
	// wait for it to get reset
	while(*CPUCFG_CNT64_CTRL_REG & (1 << 1))
	{
	}

	// return counter value
	cnt = (((uint64)(*CPUCFG_CNT64_HIGH_REG)) << 32) | *CPUCFG_CNT64_LOW_REG;
	//mutex_exit(&sc->sc_lock);

	return cnt;
}

uint32 get_cpsr(void)
{
	uint32 retval;
	asm volatile (" mrs  %0, cpsr" : "=r" (retval) : /* no inputs */);
	return retval;
}

void set_cpsr(uint32 val)
{
	asm volatile (" msr  cpsr_c, %0" : /* no outputs */ : "r" (val));
}

uint64 diff_cycles(uint32 cyc_old, uint32 cyc_new)
{
	if (cyc_new < cyc_old)
	{
		return (0xffffffff - cyc_old) + cyc_new;
	}
	else
	{
		return cyc_new - cyc_old;
	}
}

void traceCore()
{
	uint32 cid = getcid();

	// store cid in sram (write/read access test)
	h3_spinlock_acquire(1);
	volatile uint32* p = (uint32 *)0x4;
	*p = cid;
	h3_spinlock_release(1);

	struct cpucfg_csreg* cpucfg;
	cpucfg = (struct cpucfg_csreg*)CPUCFG_BASE;

	// print heartbeat trace
	uint32 cnt64 = get_cnt64();
	uint32 cnt64_low = (uint32)(cnt64 & 0xfffffff);
	uint32 cnt64_ms = (uint32)((cnt64 / 24000) & 0xffffff);
	uint32 cycle = get_ccnt();
	kprintf("core%u sp=0x%x cycle=%u cnt64_low=%u cnt64_ms=%u sctlr=0x%x actlr=0x%x ttbr0=0x%x ttbcr=0x%x gc=0x%08x\r\n", 
			cid, &cid, cycle, cnt64_low, cnt64_ms, get_sctlr(), get_actlr(), get_ttbr0(), get_ttbcr(), cpucfg->genctrl);
	kprintf("core%u cpsr=0x%x scr=0x%08x prrr=0x%08x nmrr=9x%08x pmcr=0x%08x freq=%u\r\n", 
			cid, get_cpsr(), get_scr(), get_prrr(), get_nmrr(), get_pmcr(), get_cpuclk());
}

/*------------------------------------------------------------------------
 *  cpuinit  -  Initialize CPUs
 *------------------------------------------------------------------------
 */
void cpuinit(void){
	uint32 i;				/* iterator over cores */
	struct cpuent* cpuptr;	/* pointer to cpu entry */

    led_init();
	h3_spinlock_init();
	perfcounters_init();

//    kprintf("Hello from core %d! (sp=0x%x f=%u)\r\n", getcid(), &i, get_cpuclk());
	traceCore();

	/* set secondary entry point */
	cpu_set_entry(secondary_start);

	for(i = 1; i < NCPU; i++){
		cpuptr = &cputab[i];

		/* Scheduling is not currently blocked */
		cpuptr->defer.ndefers = 0;
		cpuptr->defer.attempt = FALSE;

		/* Initialize current and previous processes */
		cpuptr->cpid = i;
		cpuptr->ppid = i;

		/* Set initial preemption time */
		cpuptr->preempt = 1000;

		/* wake up auxiliary cores */
		cpu_enable(i);
	}

}

/*------------------------------------------------------------------------
 *  getcid  -  Return the ID of the currently executing core
 *------------------------------------------------------------------------
 */
cid32 getcid(void){
	cid32 cid;
	/* Read Multiprocessor Affinity Register */
	asm volatile ( "MRC p15, 0, %0, c0, c0, 5\t\n": "=r"(cid) );
	cid &= MPIDR_CID; /* mask off CPU ID bits */
	return cid;
}

/*------------------------------------------------------------------------
 *  cpu_sev  -  release other cpus from wfe state
 *------------------------------------------------------------------------
 */
void cpu_sev(void){
	asm volatile("sev");
}

/*------------------------------------------------------------------------
 *  cpu_wfe  -  wait for event
 *------------------------------------------------------------------------
 */
void cpu_wfe(void){
	asm volatile("wfe");
}

void power_on_core(uint32 cpu)
{
	struct cpucfg_csreg* cpucfg = (struct cpucfg_csreg*)CPUCFG_BASE;

	// set core entry point
	cpucfg->pcstart = (uint32)secondary_start;
	dsb();

	// assert core reset
	cpucfg->cpu[cpu].rstctrl = 0;
	dsb();

	// reset L1 cache
	cpucfg->genctrl &= ~(1 << cpu);
	dsb();

	// disable debug access
	cpucfg->dbg_ctrl1 &= ~(1 << cpu);
	dsb();

	// release power clamp (Linux does it this way and U-boot similar)
	struct prcm_reg* prcm = (struct prcm_reg*)PRCM_BASE;
	prcm->cpu_pwr_clamp[cpu] = 0xFE;
	dsb();
	MDELAY(10); // Linux waits 10ms
	prcm->cpu_pwr_clamp[cpu] = 0xF8;
	dsb();
	MDELAY(10);
	prcm->cpu_pwr_clamp[cpu] = 0xE0;
	dsb();
	MDELAY(10);
	prcm->cpu_pwr_clamp[cpu] = 0x80;
	dsb();
	MDELAY(10);
	prcm->cpu_pwr_clamp[cpu] = 0x00;
	MDELAY(10);
	dsb();
	while (prcm->cpu_pwr_clamp[cpu] != 0)
	{
	}
	MDELAY(20); // Linux waits 20ms

				// clear power gating
	prcm->cpu_pwroff &= ~(1 << cpu);
	dsb();

	// deassert core reset
	cpucfg->cpu[cpu].rstctrl = 3;
	dsb();

	// enable debug access
	cpucfg->dbg_ctrl1 |= (1 << cpu);
	dsb();
}

/*------------------------------------------------------------------------
 *  cpu_enable  -  reset and turn on given core
 *------------------------------------------------------------------------
 */
status cpu_enable(cid32 cid){

	kprintf("starting core%u\r\n", cid);

	// set the startup flag
	*((volatile uint32*)0x8) = 1;
	asm volatile ("dsb");

	// power on the core
	power_on_core(cid);

	// wait for core to reach secondary_run() below
	// where it resets the startup flag
	uint32 down = 1;
	while (down)
	{
		down = *((volatile uint32*)0x8);
		asm volatile ("dsb");
	}

	kprintf("core%d started\r\n", cid);

	return OK;
}

/*------------------------------------------------------------------------
 *  secondary_run  -  get already initialized and running secondary core
 *                    integrated and running Xinu processes
 *------------------------------------------------------------------------
 */
void secondary_run(void)
{
	// signal started to core0 in cpu_enable() above
	*((volatile uint32*)0x8) = 0;

	// enable performance counters on this core
	perfcounters_init();

	//gicinit();
	//enable();

	// blink both LEDs
	int nLedOn = 0;
	while (1)
	{
		if (!nLedOn)
		{
			if (getcid() == 1)
			{
				pwr_on();
			}
			else
			{
				status_on();

			}
			nLedOn = 1;
		}
		else
		{
			if (getcid() == 1)
			{
				pwr_off();
			}
			else
			{
				status_off();

			}
			nLedOn = 0;
		}
		//kprintf("Hello from core %d! (sp=0x%x f=%u)\r\n", getcid(), &p, get_cpuclk());
		traceCore();
		MDELAY(20000);
	}

	prnull();
}

/*------------------------------------------------------------------------
*  cpu_set_entry  -  set cpu entry point
*------------------------------------------------------------------------
*/
void cpu_set_entry(void* entry) {
	struct cpucfg_csreg* cpucfg = (struct cpucfg_csreg*)CPUCFG_BASE;
	cpucfg->pcstart = (uint32)entry;
}

/*------------------------------------------------------------------------
 *  printreg  -  print out name, address, and contents of device register
 *------------------------------------------------------------------------
 */
void printreg(char* name, reg32* addr){
	kprintf("%27s:\t0x%08X\t0x%08X\n", name, addr, *addr);
}

/*------------------------------------------------------------------------
 *  cpu_dump  -  dump contents of cpu csrs
 *------------------------------------------------------------------------
 */
void cpu_dump(void){
	int i;
	static int num = 1;
	struct cpucfg_csreg* cpucfg = (struct cpucfg_csreg*)CPUCFG_BASE;

	kprintf("********************* CPU CSR DUMP %d *********************\n", num++);

	kprintf("%28s\t%8s\t%8s\n", "Register", "Address", "Value");
	kprintf("----------------------------\t");
	kprintf("--------\t");
	kprintf("--------\n");
	printreg(	"rstctlall", 		&cpucfg->rstctlall);
	for(i = 0; i < 4; i++){
		kprintf("\t      CPU %d RstCtrl:\t0x%08X\t0x%08X\n", i, &cpucfg->cpu[i].rstctrl, cpucfg->cpu[i].rstctrl);
		kprintf("\t      CPU %d Control:\t0x%08X\t0x%08X\n", i, &cpucfg->cpu[i].ctrl,cpucfg->cpu[i].ctrl);
		kprintf("\t      CPU %d  Status:\t0x%08X\t0x%08X\n", i, &cpucfg->cpu[i].stat,cpucfg->cpu[i].stat);
	}
	printreg(	"sysrst", 						&cpucfg->sysrst		);
	printreg(	"clock gating", 				&cpucfg->clkgating	);
	printreg(	"general control", 				&cpucfg->genctrl	);
	printreg(	"super standby flag", 			&cpucfg->supstanflg	);
	printreg(	"secondary entry", 				&cpucfg->pcstart	);
	printreg(	"private 1", 					&cpucfg->private1	);
	printreg(	"counter control", 				&cpucfg->cnt64ctrl	);
	printreg(	"counter low", 					&cpucfg->cnt64low	);
	printreg(	"counter high", 				&cpucfg->cnt64high	);
	
	kprintf("**********************************************************\n");
}

// TODO - SCU
void cpu_set_acinactm()
{
	struct cpucfg_csreg* cpucfg = (struct cpucfg_csreg*)CPUCFG_BASE;
	cpucfg->genctrl |= (1 << 6);
	asm volatile ("dsb");
	asm volatile ("isb");
	asm volatile ("dmb");
}

// TODO - SCU
void cpu_reset_acinactm()
{
	struct cpucfg_csreg* cpucfg = (struct cpucfg_csreg*)CPUCFG_BASE;
	cpucfg->genctrl &= ~(1 << 6);
	asm volatile ("dsb");
	asm volatile ("isb");
	asm volatile ("dmb");
}











#define GPIO_A    0
#define GPIO_B    1
#define GPIO_C    2
#define GPIO_D    3
#define GPIO_E    4
#define GPIO_F    5
#define GPIO_G    6
#define GPIO_H    7
#define GPIO_I    8
#define GPIO_L    9	/* R_PIO */

struct h3_gpio {
	volatile unsigned long config[4];
	volatile unsigned long data;
	volatile unsigned long drive[2];
	volatile unsigned long pull[2];
};

/* In theory each gpio has 32 pins, but they are actually populated like so.
*/
// static int gpio_count[] = { 22, 0, 19, 18, 16, 7, 14, 0, 0, 12 };

static struct h3_gpio * gpio_base[] = {
	(struct h3_gpio *) 0x01C20800,		/* GPIO_A */
	(struct h3_gpio *) 0x01C20824,		/* GPIO_B */
	(struct h3_gpio *) 0x01C20848,		/* GPIO_C */
	(struct h3_gpio *) 0x01C2086C,		/* GPIO_D */
	(struct h3_gpio *) 0x01C20890,		/* GPIO_E */
	(struct h3_gpio *) 0x01C208B4,		/* GPIO_F */
	(struct h3_gpio *) 0x01C208D8,		/* GPIO_G */
	(struct h3_gpio *) 0x01C208FC,		/* GPIO_H */
	(struct h3_gpio *) 0x01C20920,		/* GPIO_I */

	(struct h3_gpio *) 0x01F02c00,		/* GPIO_L */
};

/* Only A, G, and R can interrupt */

/* GPIO pin function config (0-7) */
#define GPIO_INPUT        (0)
#define GPIO_OUTPUT       (1)
#define H3_GPA_UART0      (2)
#define H5_GPA_UART0      (2)
#define GPIO_DISABLE      (7)

/* GPIO pin pull-up/down config (0-3)*/
#define GPIO_PULL_DISABLE	(0)
#define GPIO_PULL_UP		(1)
#define GPIO_PULL_DOWN		(2)
#define GPIO_PULL_RESERVED	(3)

/* There are 4 config registers,
* each with 8 fields of 4 bits.
*/
void
gpio_config(int gpio, int pin, int val)
{
	struct h3_gpio *gp = gpio_base[gpio];
	int reg = pin / 8;
	int shift = (pin & 0x7) * 4;
	int tmp;

	tmp = gp->config[reg] & ~(0xf << shift);
	gp->config[reg] = tmp | (val << shift);
}

/* There are two pull registers,
* each with 16 fields of 2 bits.
*/
void
gpio_pull(int gpio, int pin, int val)
{
	struct h3_gpio *gp = gpio_base[gpio];
	int reg = pin / 16;
	int shift = (pin & 0xf) * 2;
	int tmp;

	tmp = gp->pull[reg] & ~(0x3 << shift);
	gp->pull[reg] = tmp | (val << shift);
}

void
gpio_output(int gpio, int pin, int val)
{
	struct h3_gpio *gp = gpio_base[gpio];

	if (val)
		gp->data |= 1 << pin;
	else
		gp->data &= ~(1 << pin);
}

int
gpio_input(int gpio, int pin)
{
	struct h3_gpio *gp = gpio_base[gpio];

	return (gp->data >> pin) & 1;
}

void
uart_gpio_init(void)
{
	gpio_config(GPIO_A, 4, H3_GPA_UART0);
	gpio_config(GPIO_A, 5, H3_GPA_UART0);
	gpio_pull(GPIO_A, 5, GPIO_PULL_UP);
}

#define POWER_PIN	10
#define STATUS_PIN	15	/* Orange Pi */

void
led_init(void)
{
	gpio_config(GPIO_L, POWER_PIN, GPIO_OUTPUT);
	gpio_config(GPIO_A, STATUS_PIN, GPIO_OUTPUT);
}

/* This is the green LED */
void
pwr_on(void)
{
	gpio_output(GPIO_L, POWER_PIN, 1);
}

void
pwr_off(void)
{
	gpio_output(GPIO_L, POWER_PIN, 0);
}

/* This is the red LED */
void
status_on(void)
{
	gpio_output(GPIO_A, STATUS_PIN, 1);
}

void
status_off(void)
{
	gpio_output(GPIO_A, STATUS_PIN, 0);
}

/* A reasonable delay for blinking an LED
* (at least it is if the D cache is enabled)
*/
static void
__delay_blink(void)
{
	// volatile int count = 50000000;
	volatile int count = 500000;

	//printf ( "Start delay\n" );
	while (count--)
		;
	//printf ( "End delay\n" );
}

/* Blink red status light */
void
gpio_blink_red(void)
{
	for (;; ) {
		//printf ( "Red on\n" );
		status_on();
		__delay_blink();

		// printf ( "Red off\n" );
		status_off();
		__delay_blink();
	}
}

/* Blink green "power" light */
void
gpio_blink_green(void)
{
	for (;; ) {
		pwr_on();
		__delay_blink();

		pwr_off();
		__delay_blink();
	}
}
