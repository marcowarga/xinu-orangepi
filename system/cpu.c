/*  cpu.c */
#include <xinu.h>
#include <h3spinlock.h>

extern uint32 exp_vector[];

#define PMCR_ENABLE	0x01	/* enable all counters */
#define PMCR_RESET	0x02	/* reset all counters */
#define PMCR_CC_RESET	0x04	/* reset CCNT */
#define PMCR_CC_DIV	0x08	/* divide CCNT by 64 */
#define PMCR_EXPORT	0x10	/* export events */
#define PMCR_CC_DISABLE	0x20	/* disable CCNT in non-invasive regions */

/* There are 4 counters besides the CCNT (we ignore them at present) */
#define CENA_CCNT	0x80000000
#define CENA_CTR3	0x00000008
#define CENA_CTR2	0x00000004
#define CENA_CTR1	0x00000002
#define CENA_CTR0	0x00000001

uint32 get_pmcr();
uint32 get_ccnt();
void set_pmcr(uint32);
void set_cena(uint32);

uint32 get_sctlr();
uint32 get_actlr();
uint32 get_ttbr0();
uint32 get_ttbcr();

void led_init(void);
void pwr_on(void);
void pwr_off(void);
void status_on(void);
void status_off(void);
void gpio_blink_red(void);
void gpio_blink_green(void);

uint32 readTestReg(void)
{
// these read the same non-zero values on all cores
//    return *((uint32*)0x01c20800); // port A config reg 0
//    return *((uint32*)0x01c20000); // PLL_CPUX control reg
//    return *((uint32*)0x01f00000); // LOSC control reg
//    return *((uint32*)0x01c21800); // ADC control reg
    return *((uint32*)0x01f02c00); // port L config reg 0

// these read non-zero on core0 and zero on core1 - core3 
//    return *((uint32*)(0x01f01c00 + 0x48)); // CPU0 status reg
//    return *((uint32*)0x01c1e000); // SMC config reg
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
#define CPUCFG_CLK_GATING_REG ((volatile uint32*)(CPUCFG_REG_BASE + 0x0144))
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

uint32 get_cnt64_low()
{
    return (uint32)(get_cnt64() & 0xffffffff);
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

    // print heartbeat trace
    uint32 start = get_cnt64_low();
    uint32 cycstart = get_ccnt();
    uint32 diff = get_cnt64_low() - start;
    uint32 cycdiff = diff_cycles(cycstart, get_ccnt());
    kprintf("core%u sp=0x%x cnt64_diff=%u cnt64_ms=%u cyc_diff=%u sctlr=0x%x actlr=0x%x ttbr0=0x%x ttbcr=0x%x *p=0x%x cnt64_low=%u testreg=0x%08x\r\n", 
            cid, &cid, diff, diff / 24000, cycdiff, get_sctlr(), get_actlr(), get_ttbr0(), get_ttbcr(), *p, get_cnt64_low(), readTestReg());
}

/*------------------------------------------------------------------------
 *  cpuinit  -  Initialize CPUs
 *------------------------------------------------------------------------
 */
void cpuinit(void){
	uint32 i;				/* iterator over cores */
	struct cpuent* cpuptr;	/* pointer to cpu entry */

    h3_spinlock_init();
    led_init();
//    kprintf("Hello from core %d! (sp=0x%x f=%u)\r\n", getcid(), &i, get_cpuclk());
    traceCore();

	//TODO: set csrs
	//cpucfg->genctrl = 0x40; /* set snoop interface active */
	//cpucfg->genctrl |= 0x20; /* apply reset to shared L2 mem controller */

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

/*------------------------------------------------------------------------
 *  cpu_enable  -  reset and turn on given core
 *------------------------------------------------------------------------
 */
status cpu_enable(cid32 cid){
	struct cpucfg_csreg* cpucfg;
	struct cpu_csreg* cpureg;
	uint32 cpumask;

	/* Check for valid core */
	if(isbadcid(cid)){ return SYSERR; }

    kprintf("starting core %d\r\n", cid);

    *((volatile uint32*)0x8) = 1;

	/* Get core csrs and mask */
	cpucfg = (struct cpucfg_csreg*)CPUCFG_BASE;
	cpureg = &cpucfg->cpu[cid];
	cpumask = (1 << cid);

	/* assert cpu core reset */
	//cpureg->rstctrl &= ~CPU_CORE_RST;
    cpureg->rstctrl = 0;

	/* L1RSTDISABLE hold low TODO: need this sill? */
	cpucfg->genctrl &= ~cpumask;

#define PRCM_BASE 0x01f01400
#define PWR_OFF_REG ((volatile uint32*)(PRCM_BASE + 0x100))
    *PWR_OFF_REG &= ~cpumask;
    MDELAY(2000);

    /* de-assert core reset */
	cpureg->rstctrl = 3;

    // wait for core to start
    while (*((volatile uint32*)0x8))
    {
    }

    kprintf("core %d started\r\n", cid);

	return OK;
}

/*------------------------------------------------------------------------
 *  cpu_set_entry  -  set cpu entry point
 *------------------------------------------------------------------------
 */
void cpu_set_entry(void* entry){
	struct cpucfg_csreg* cpucfg = (struct cpucfg_csreg*)CPUCFG_BASE;
	cpucfg->pcstart = (uint32)entry;
}

/*------------------------------------------------------------------------
 *  secondary_run  -  get already initialized and running secondary core
 *                    integrated and running Xinu processes
 *------------------------------------------------------------------------
 */
void secondary_run(void)
{
    // signal started
    *((volatile uint32*)0x8) = 0;

    //volatile void* p = 0;

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
