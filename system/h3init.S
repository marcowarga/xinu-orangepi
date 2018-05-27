/*  $NetBSD: a9_mpsubr.S,v 1.55 2018/01/18 12:49:09 skrll Exp $ */
/*-
 * Copyright (c) 2012 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

.equ CPU_CONTROL_DC_ENABLE, 0x00000004
.equ CPU_CONTROL_IC_ENABLE, 0x00001000
.equ CPU_CONTROL_MMU_ENABLE, 0x00000001
.equ CORTEXA9_AUXCTL_SMP, 0x00000040
.equ PSR_SVC32_MODE, 0x00000013

.text

.globl coretex_init
coretex_init:
	mov r10, lr             // save lr

	cpsid   if, #PSR_SVC32_MODE     // SVC32 with no interrupts
	
	mov r0, #0
	msr spsr_sxc, r0            // set SPSR[23:8] to known value
	
	mrc	p15, 0, r4, c1, c0, 0		// SCTLR read

	// If SMP is already enabled, don't do anything (maybe).
	//
	mrc p15, 0, r0, c1, c0, 1       // ACTLR read
	orr r1, r0, #CORTEXA9_AUXCTL_SMP    // test SMP

	cmp r0, r1              // ACTLR have SMP+<31> set?
	bxeq    r10             // return if set

	//
	// Before turning on SMP, turn off the caches and the MMU.
	//
	dsb
	movw	r1,#(CPU_CONTROL_IC_ENABLE | CPU_CONTROL_DC_ENABLE | CPU_CONTROL_MMU_ENABLE)
	bic	r0, r4, r1			// disable icache/dcache/mmu
	mcr	p15, 0, r0, c1, c0, 0		// SCTLR write
	dsb
	isb

	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		// toss i-cache

	//
	// Step 1b, invalidate the data cache
	//
	//XPUTC(#'B')
	bl armv7_dcache_wbinv_all
	//XPUTC(#'C')

	//
	// The MMU is off.  Make sure the TLB is invalidated before
	// turning on SMP.
	//
	mov	r0, #0
	mcr	p15, 0, r0, c8, c7, 0	// TLBIALL (just this core)

	// For the A7, SMP must be on ldrex/strex to work.
	//
	//
	// Step 4a, set ACTLR.SMP=1, if ACTLR.SMP=0
	// i.e. OMAP4430 was enabled yet.
	//
	mrc	p15, 0, r0, c1, c0, 1		// ACTLR read
	tst	r0, #CORTEXA9_AUXCTL_SMP
	orreq	r0, r0, #CORTEXA9_AUXCTL_SMP	// enable SMP
	mcreq	p15, 0, r0, c1, c0, 1		// ACTLR write
	isb
	dsb

	//
	// Step 4b, restore SCTLR (enable the data cache)
	//
	orr	r4, r4, #CPU_CONTROL_IC_ENABLE	// enable icache
	orr	r4, r4, #CPU_CONTROL_DC_ENABLE	// enable dcache
	mcr	p15, 0, r4, c1, c0, 0		// SCTLR write
	isb
	//XPUTC(#'-')

	bx r10
