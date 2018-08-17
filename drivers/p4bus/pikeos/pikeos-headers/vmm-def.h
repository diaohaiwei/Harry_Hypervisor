/*
 *  PikeOS VMM protocol definition
 *
 *  Copyright (C) 2014, SYSGO AG
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */

#ifndef _VMM_DEF_H
#define _VMM_DEF_H

/* ------------------------- FILE INCLUSION -------------------------------- */

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

/**
 * Current version: 0x0005
 *
 * History:
 *  0x0001: First version
 *  0x0002: Introduction of the p4bus interface
 *  0x0003: Stable interface version for PikeOS 3.5
 *  0x0004: Addition of the stat device (bus discovery)
 *  0x0005: Major rework of API: Unification with TrustZone and
 *          renamed to VMM.
 *  0x0010: Removing of vmm_signaling, interrupt per device.
 *  0x0011: VMM message size 4, no interrupt per device
 *  0x0012: Use of hvc #10 for vmm call, use HVC #99 for debug call
 */
#define VMM_API_VERSION     0x0012

/**
 * Number of HVC to use for VMM calls
 */
#define VMM_HVC_NUMBER		10

/**
 * Number of HVC to use for Debug calls
 */
#define VMM_HVC_DEBUG		99

/**
 * Length of a VMM message
 */
#define VMM_MESSAGE_SIZE    4

/**
 * Data size in a VMM message
 */
#define VMM_DATA_SIZE       3

/**
 * VMM Bypass definitions
 */

#define VMM_BYPASS_READ			(0<<16)
#define VMM_BYPASS_WRITE		(1<<16)
#define VMM_BYPASS_TYPE(val)	(val & 0xffff0000)

#define VMM_BYPASS_8BIT		0
#define VMM_BYPASS_16BIT	1
#define VMM_BYPASS_32BIT	2
#define VMM_BYPASS_64BIT	3
#define VMM_BYPASS_SIZE(val)	(val & 0xffff)

/* ------------------------ TYPE DECLARATIONS ------------------------------ */
/**
 * vmm_dev_id_t defines the VMM devices available by
 * calling 'hvc VMM_HVC_NUMBER' with the 4 first registers as parameters
 * r0/x0: device ID (vmm_dev_id_t)
 * r1/x1: arg[0]
 * r2/x2: arg[1]
 * r3/x3: arg[2]
 */

/**
 * Those are the possible VMM operations.
 * Each operation has its own devices defining the
 * protocol to be used.
 *
 * VMM_CUSTOM is the place to add new devices without
 * getting in conflict with standard devices.
 */
typedef enum vmm_dev_id_e {
	/**
	 * VMM_DEV_INFO:
	 * This gives back information (memory size, number of cpu, version etc)
	 * arg[0] must be the info number comming from vmm_info_op_t
	 */
	VMM_DEV_INFO = 0,
	/**
	 * VMM_DEV_EARLYCON:
	 * This is a vmm call providing an early console for
	 * a guest.
	 * If arg[0] is 0, an exchange memory is initialised
	 * with arg[1] guest physical address.
	 * If arg[0] is not 0, arg[0] characters from the exchange area
	 * are printed on PikeOS console.
	 *
	 * This is a blocking VMM call handling prints directly and
	 * as such should not be used after init of a guest.
	 */
	VMM_DEV_EARLYCON = 1,
	/**
	 * VMM_DEV_BYPASS:
	 * This is a vmm call providing access to IO areas through
	 * the manager.
	 * arg[0] must contain the access type and size (see VMM_BYPASS defines)
	 * arg[1] must contain the physical address
	 * arg[2] must contain the value to write (64 bit access only supported in 64bit)
	 *
	 * Only physical areas defined in the configuration are available.
	 * ! Access are done in user mode in PikeOS !
	 */
	VMM_DEV_BYPASS = 2,
	/**
	 * VMM_DEV_P4BUS:
	 * This is a vmm call providing access to the P4BUS
	 * arg[0]: P4BUS command
	 * arg[1]: command argument (see p4bus-def.h)
     * arg[2]: command argument (see p4bus-def.h)
	 */
	VMM_DEV_P4BUS = 3,
	/**
	 * VMM_DEV_VMAPI:
	 * This is a vmm call providing access to some vmapi like
	 * reboot/stop partition or target, change sched scheme
	 *
	 * See vmm_vmapi_op_t for more information
	 */
	VMM_DEV_VMAPI = 4,
} vmm_dev_id_t;

/**
 * @purpose Enumeration of VMM info command
 *
 * When the guest sends a VMM info command there are 4 values in
 * the VMM message:
 * - devid : VMM_DEV_INFO
 * - arg[0]: msg[0] VMM info command ID (vmm_info_op_t)
 * - arg[1]: msg[1]
 * - arg[2]: msg[2]
 */
typedef enum vmm_info_op_e {
    /**
     *  @purpose
     *    Returns the VMM API and PikeOS API version
     *
     *  @param msg[0] VMM_INFO_VERSION
     *  @param msg[1] not used
     *  @param msg[2] not used
     * 
     *  @returns
     *    Return the API version numbers
     *  @retval msg[0]:   VMM_API_VERSION
     *  @retval msg[1]:   P4_API_VERSION
     *  @retval msg[2]:   not used
     */
	VMM_INFO_VERSION = 0,
    /**
     *  @purpose
     *    Returns information on the guest system
     *
     *  @param msg[0] VMM_INFO_SYSTEM
     *  @param msg[1] not used
     *  @param msg[2] not used
     * 
     *  @returns
     *    Return the API version numbers
     *  @retval msg[0]:   Memory size in bytes
     *  @retval msg[1]:   Number of cores
     *  @retval msg[2]:   Timer clock frequency
     */
	VMM_INFO_SYSTEM = 1,
} vmm_info_op_t;

/**
 * @purpose Enumeration of VMAPI operations
 *
 * When the guest sends a VMAPI operation there are 4 values in
 * the VMM message:
 * - devid : VMM_DEV_VMAPI
 * - arg[0]: msg[0]  VMAPI operation (vmm_vmapi_op_t)
 * - arg[1]: msg[1]
 * - arg[2]: msg[2]
 */
typedef enum vmm_vmapi_op_e {
    /**
     *  @purpose
     *    reboot a partition on the system
     *
     *  @param msg[0] VMM_VMAPI_REBOOT_PARTITION
     *  @param msg[1] Partition ID
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
	VMM_VMAPI_REBOOT_PARTITION = 0,
    /**
     *  @purpose
     *    reboot the complete target
     *
     *  @param msg[0] VMM_VMAPI_REBOOT_TARGET
     *  @param msg[1] not used
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
	VMM_VMAPI_REBOOT_TARGET = 1,
    /**
     *  @purpose
     *    Stop a partition
     *
     *  @param msg[0] VMM_VMAPI_STOP_PARTITION
     *  @param msg[1] partition ID
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
	VMM_VMAPI_STOP_PARTITION = 2,
    /**
     *  @purpose
     *    Stop the complete target
     *
     *  @param msg[0] VMM_VMAPI_STOP_TARGET
     *  @param msg[1] not used
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
	VMM_VMAPI_STOP_TARGET = 3,
    /**
     *  @purpose
     *    set the time scheduling scheme
     *
     *  @param msg[0] VMM_VMAPI_SET_TIME_SCHED
     *  @param msg[1] physical address of the string where is set the
     *                name of the scheduling scheme to switch to
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
	VMM_VMAPI_SET_TIME_SCHED = 4,
} vmm_vmapi_op_t;

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

/* ----------------------- FUNCTION DECLARATIONS --------------------------- */

#endif /* _VMM_DEF_H */
