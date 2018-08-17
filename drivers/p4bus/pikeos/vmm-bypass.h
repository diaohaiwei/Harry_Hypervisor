/*
 *  PikeOS VMM Bypass interface
 * 
 *  Copyright (C) 2014, SYSGO AG
 * 
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */
#ifndef VMM_BYPASS_H
#define VMM_BYPASS_H

/* ------------------------- FILE INCLUSION -------------------------------- */

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

/* ----------------------- FUNCTION DECLARATIONS --------------------------- */

extern int vmm_bypass_detected(void);

/**
 * Those functions will do a read through the vmm bypass driver
 * The PikeOS Host must have areas configured for the corresponding
 * physical address.
 *
 * Address passed to those functions must be a virtual address
 * in the Linux Kernel.
 */
extern uint8_t  vmm_bypass_read8(unsigned long addr);
extern uint16_t vmm_bypass_read16(unsigned long addr);
extern uint32_t vmm_bypass_read32(unsigned long addr);
extern uint64_t vmm_bypass_read64(unsigned long addr);

/**
 * Those functions will do a write through the vmm bypass driver
 * The PikeOS Host must have areas configured for the corresponding
 * physical address and not in real only mode.
 *
 * Address passed to those functions must be a virtual address
 * in the Linux Kernel.
 */
extern void vmm_bypass_write8(unsigned long addr, uint8_t value);
extern void vmm_bypass_write16(unsigned long addr, uint16_t value);
extern void vmm_bypass_write32(unsigned long addr, uint32_t value);
extern void vmm_bypass_write64(unsigned long addr, uint64_t value);

#endif
