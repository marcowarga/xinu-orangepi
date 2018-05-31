/* cpu.h */

#define NCPU 4	/* number of cores in SMP system */

/* CPU Config Control and Status Registers */

#define CPUCFG_BASE 0x01F01C00	/* Base address */
#define PRCM_BASE 0x01f01400

#define CPU_CORE_RST 0x00000002
#define CPU_RST 0x00000001

/* csr for individual cpu */
struct cpu_csreg {
	reg32 rstctrl;		/* reset control register */
	reg32 ctrl;			/* control register */
	reg32 stat;			/* status register */
	reg32 pad[13];		/* padding */
};

/* csr config bank for all cpus */
struct cpucfg_csreg {
	reg32 rstctlall;			/* reset control register for all cpus */
	reg32 pad1[15];				/* padding */
	struct cpu_csreg cpu[4];	/* individual csr's for each cpu */
	reg32 sysrst;				/* cpu system reset register */
	reg32 clkgating;			/* cpu clock gating register */
	reg32 pad2[15];				/* padding */
	reg32 genctrl;				/* general control register */
	reg32 pad3[6];				/* padding */
	reg32 supstanflg;			/* super standby flag register */
	reg32 pcstart;				/* program counter start register */
	reg32 private1;				/* program counter start register */
    reg8 pad4[0x4];				/* 0x1ac */
	reg32 cpu1_pwr_clamp;		/* 0x1b0 sun7i only */
	reg32 cpu1_pwroff;			/* 0x1b4 sun7i only */
	reg8 pad5[0x2c];			/* 0x1b8 */
	reg32 dbg_ctrl1;			/* 0x1e4 */
	reg8 pad6[0x18];			/* 0x1e8 */
	reg32 idle_cnt0_low;		/* 0x200 */
	reg32 idle_cnt0_high;		/* 0x204 */
	reg32 idle_cnt0_ctrl;		/* 0x208 */
	reg8 pad7[0x4];				/* 0x20c */
	reg32 idle_cnt1_low;		/* 0x210 */
	reg32 idle_cnt1_high;		/* 0x214 */
	reg32 idle_cnt1_ctrl;		/* 0x218 */
	reg8 pad8[0x4];				/* 0x21c */
	reg32 idle_cnt2_low;		/* 0x220 */
	reg32 idle_cnt2_high;		/* 0x224 */
	reg32 idle_cnt2_ctrl;		/* 0x228 */
	reg8 pad10[0x4];			/* 0x22c */
	reg32 idle_cnt3_low;		/* 0x230 */
	reg32 idle_cnt3_high;		/* 0x234 */
	reg32 idle_cnt3_ctrl;		/* 0x238 */
	reg8 pad11[0x4];			/* 0x23c */
	reg32 idle_cnt4_low;		/* 0x240 */
	reg32 idle_cnt4_high;		/* 0x244 */
	reg32 idle_cnt4_ctrl;		/* 0x248 */
	reg8 pad12[0x34];			/* padding */
	reg32 cnt64ctrl;			/* 64-bit counter control register */
	reg32 cnt64low;				/* 64-bit counter low register */
	reg32 cnt64high;			/* 64-bit counter high register */
};

struct prcm_reg {
	reg32 cpus_cfg;			/* 0x000 */
	reg8 res0[0x8];			/* 0x004 */
	reg32 apb0_ratio;		/* 0x00c */
	reg32 cpu0_cfg;			/* 0x010 */
	reg32 cpu1_cfg;			/* 0x014 */
	reg32 cpu2_cfg;			/* 0x018 */
	reg32 cpu3_cfg;			/* 0x01c */
	reg8 res1[0x8];			/* 0x020 */
	reg32 apb0_gate;		/* 0x028 */
	reg8 res2[0x14];		/* 0x02c */
	reg32 pll_ctrl0;		/* 0x040 */
	reg32 pll_ctrl1;		/* 0x044 */
	reg8 res3[0x8];			/* 0x048 */
	reg32 clk_1wire;		/* 0x050 */
	reg32 clk_ir;			/* 0x054 */
	reg8 res4[0x58];		/* 0x058 */
	reg32 apb0_reset;		/* 0x0b0 */
	reg8 res5[0x3c];		/* 0x0b4 */
	reg32 clk_outd;			/* 0x0f0 */
	reg8 res6[0xc];			/* 0x0f4 */
	reg32 cpu_pwroff;		/* 0x100 */
	reg8 res7[0xc];			/* 0x104 */
	reg32 vdd_sys_pwroff;	/* 0x110 */
	reg8 res8[0x4];			/* 0x114 */
	reg32 gpu_pwroff;		/* 0x118 */
	reg8 res9[0x4];			/* 0x11c */
	reg32 vdd_pwr_reset;	/* 0x120 */
	reg8 res10[0x1c];		/* 0x124 */
	reg32 cpu_pwr_clamp[4];	/* 0x140 but first one is actually unused */
	reg8 res11[0x30];		/* 0x150 */
	reg32 dram_pwr;			/* 0x180 */
	reg8 res12[0xc];		/* 0x184 */
	reg32 dram_tst;			/* 0x190 */
};

/* Hold current pid executing on each processor */

struct cpuent {
	pid32 cpid;					/* ID of currently executing process */
	pid32 ppid;					/* ID of previously executing process */
	struct deferent defer;		/* Deferred scheduling for cpu */
	uint32 preempt;				/* count 1000 ms for cpu */ 
	int32 cpidpad[11];			/* Pad to size of ERG to avoid false sharing */
};

extern 	struct	cpuent	cputab[];

#define CPU_NONE	-1
#define CPU_ALL		-2

#define isbadcid(x)	(x < 0 || x >= NCPU)

