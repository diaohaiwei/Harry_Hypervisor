/*
 *  PikeOS P4 bus interface
 * 
 *  Copyright (C) 2014, SYSGO AG
 * 
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */

#ifndef VMM_H
#define VMM_H

/* ------------------------- FILE INCLUSION -------------------------------- */

#include "compat.h"
#include "pikeos-headers/vmm-def.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define TEXTIFY(A) #A

#if VMM_MESSAGE_SIZE != 4
#error VMM_MESSAGE_SIZE is not 4 anymore, you must update vmm_send_message
#endif

#if __ARM_ARCH == 7

#define REG(id)			"r" #id

#ifdef NO_VIRT_EXT
#define HYPCALL(id)		".word " TEXTIFY(0xE1400070 | (((id) & 0xFFF0) << 4) | ((id) & 0x000F))
#else
#define	HYPCALL(id)		".arch_extension virt\n" \
						"hvc #" TEXTIFY(id) "\n"
#endif
#elif __ARM_ARCH == 8

#define REG(id)			"x" #id
#define	HYPCALL(id)		"hvc #" TEXTIFY(id) "\n"

#else
#error Unsupported architecture
#endif

/* ------------------------ TYPE DECLARATIONS ------------------------------ */


/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

/* ----------------------- FUNCTION DECLARATIONS --------------------------- */

/**
 *  @purpose
 *    Register a device on the bus.
 *
 *  @pre
 *    In the given vmm device, the pdev field must be set.
 *
 *  @param device  the vmm device to register
 *  @param id      ID of the device to register
 *
 *
 *  @returns
 *  @retval 0        if register was successfull
 *  @retval -EIVNAL  if device cannot be registered
 *
 *
 *  @design
 *    Appart from opend and pdev field, the rest of the structure
 *    will be filled with information detected (devname and others).
 */
extern int vmm_detected(void);

/**
 *  @purpose
 *    Send a message on a vmm device
 *
 *  @param id    vmm device ID to make the operation on
 *  @param msg       message to be sent
 *
 *  @returns
 *  @retval VMM_E_OK          if message is successfully sent
 *
 *  @design
 *    The message is sent and an answer is received directly in the message.
 */
static inline int vmm_send_message(uint32_t devid, unsigned long *msg)
{
	register unsigned long r_id __asm__ (REG(0));
	register unsigned long r_msg0 __asm__ (REG(1));
	register unsigned long r_msg1 __asm__ (REG(2));
	register unsigned long r_msg2 __asm__ (REG(3));

	r_id = devid;
	r_msg0 = msg[0];
	r_msg1 = msg[1];
	r_msg2 = msg[2];

	__asm__ __volatile(HYPCALL(VMM_HVC_NUMBER)
						: "+r" (r_id), "+r" (r_msg0), "+r" (r_msg1), "+r" (r_msg2)
						:
						: "memory");

	msg[0] = r_msg0;
	msg[1] = r_msg1;
	msg[2] = r_msg2;

	return r_id;
}

static inline void vmm_debug(unsigned long arg0, unsigned long arg1, unsigned long arg2)
{
	register unsigned long r0 __asm__ (REG(0)) = arg0;
	register unsigned long r1 __asm__ (REG(1)) = arg1;
	register unsigned long r2 __asm__ (REG(2)) = arg2;

	__asm__ __volatile(HYPCALL(VMM_HVC_DEBUG)
						: "+r" (r0), "+r" (r1), "+r" (r2)
						:
						: "memory");
}

#endif
