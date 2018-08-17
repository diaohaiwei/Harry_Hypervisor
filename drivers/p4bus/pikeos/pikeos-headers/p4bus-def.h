/*
 *  PikeOS P4Bus protocol definition
 * 
 *  Copyright (C) 2014, SYSGO AG
 * 
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */

#ifndef P4BUS_DEF_H
#define P4BUS_DEF_H

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

/**
 * P4Bus version
 * 0x1: original version
 * 0x10: Use IORING
 */
#define P4BUS_API_VERSION	0x0010

/** host signal that at the point it went to sleep, guest must signal host if something is added in the ring */
#define OP_FLAGS_SLEEP  0x2
/** generate an irq when the operation is done */
#define OP_FLAGS_SIGNAL	0x1
/** don't signal that the operation is done, guest will poll */
#define OP_FLAGS_POLL	0x0

/**
 * Maximum number of devices handled by one driver
 */
#define P4BUS_MAX_DEVICES	256

/**
 * Maximum number of file descriptors per device
 */
#define P4BUS_MAX_FD		128

/**
 * Maximum number of iorings
 */
#define P4BUS_MAX_IORING	256

/**
 * Length of name and type of device
 */
#define P4BUS_NAMELEN	32

/**
 * GUEST IOCTL commands
 */
#define P4BUS_IOCTL_STAT			0x11
#define P4BUS_IOCTL_NATIVE			0x12

/**
 * STAT host driver capacity through IOCTL command
 */
#define PIKEOS_IOCTL_DATA_SIZE 		128
#define PIKEOS_IOCTL_STRUCT_SIZE 	(PIKEOS_IOCTL_DATA_SIZE + (3 * 4))
#define PIKEOS_IOCTL_CMD_FIELD		0
#define PIKEOS_IOCTL_SIZE_IN_FIELD	1
#define PIKEOS_IOCTL_SIZE_OUT_FIELD	2
#define PIKEOS_IOCTL_DATA_FIELD		3
#define PIKEOS_IOCTL_CMD_MASK		0xFFFF
#define PIKEOS_IOCTL_SIZE_MASK		0x7F

#define STAT_MAX_UINT32_FIELDS				16
#define PIKEOS_STAT_STRUCT_SIZE				( STAT_MAX_UINT32_FIELDS * 4 )
#define STAT_TYPE_FIELD						0
#define STAT_TOTAL_SIZE_FIELD				1
#define STAT_BLK_SIZE_FIELD					2

/* QPORT SPECIFIC */
#define STAT_QPORT_NB_MESSAGES_SRC_FIELD	3
#define STAT_QPORT_NB_MESSAGES_DST_FIELD	4
#define STAT_QPORT_DIR_FIELD				5

/* SPORT SPECIFIC */
#define STAT_SPORT_SRC_REFRESH_PERIOD_MSB	3 
#define STAT_SPORT_SRC_REFRESH_PERIOD_LSB	4 
#define STAT_SPORT_DST_REFRESH_PERIOD_MSB	5 
#define STAT_SPORT_DST_REFRESH_PERIOD_LSB	6
#define STAT_SPORT_SRC_LAST_MSG_VALIDITY	7
#define STAT_SPORT_DST_LAST_MSG_VALIDITY	8
#define STAT_SPORT_DIR_FIELD				9

#define STAT_DIR_TX							0x1
#define STAT_DIR_RX							0x2
#define STAT_DIR_RX_TX						0x3


/**
 * IOCTL macros 
 */
#define IOCTL_IN					1
#define IOCTL_OUT					0
#define P4BUS_GET_IOCTL_SIZE_IN(size) 	((size >> 16) & 0xFFFF)
#define P4BUS_GET_IOCTL_SIZE_OUT(size) 	(size & 0xFFFF)
#define P4BUS_SET_IOCTL_SIZE(size_in,size_out) 	(((size_in & 0xFFFF) << 16 ) | (size_out & 0xFFFF))

/**
 * Open flags
 */
#define P4BUS_FLAGS_OPEN_READ	0x1
#define P4BUS_FLAGS_OPEN_WRITE	0x2
#define P4BUS_FLAGS_OPEN_EXEC	0x4
#define P4BUS_FLAGS_OPEN_MAP	0x8
#define P4BUS_FLAGS_OPEN_RW		0x3
#define P4BUS_FLAGS_OPEN_RWX	0x7

/**
 * Mapping flags
 */
#define P4BUS_FLAGS_MMAP_READ	0x1
#define P4BUS_FLAGS_MMAP_WRITE	0x2
#define P4BUS_FLAGS_MMAP_EXEC	0x4
#define P4BUS_FLAGS_MMAP_RW		0x3
#define P4BUS_FLAGS_MMAP_RWX	0x7

/**
 * LSEEK OFFSET FLAGS
 */
#define P4BUS_FLAGS_SEEK_SET	0x1
#define P4BUS_FLAGS_SEEK_CUR	0x2
#define P4BUS_FLAGS_SEEK_END	0x3

/**
 * MMAPPING GRANULARITY
 */
#define MAPPING_GRANULARITY 		0x01000000UL  	// 16 Mo
#define MAPPING_MAX_MAPPED_BLOCK 	0x100		  	// 4Go / MAPPING_GRANULARITY
#define MAPPING_GRANULARITY_MASK 	(~(MAPPING_GRANULARITY - 1))  

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

/**
 * @purpose Enumeration of operation entry status
 */
typedef enum p4bus_operation_status_e {
    /** operation entry is free */
    P4BUS_OPERATION_FREE = 0,
    /** operation entry is ready to be processed */
	P4BUS_OPERATION_READY = 1,
	/** operation has been processed */
	P4BUS_OPERATION_DONE = 2,
} p4bus_operation_status_t;


/**
 * p4bus operation structure
 */
typedef struct p4bus_operation_str {
	/* status of the p4bus operation */
	volatile uint32_t status;
	/* return code of the operation */
	uint32_t retcode;
	/* operation ID */
	uint16_t type;
	/* operation flags */
	uint16_t flags;
	/* file descriptor used in a file operation */
	uint16_t filedesc;
	/* device id */
	uint16_t devid;
	/* Intermediate physical address */
	uint64_t addr;
	/* Size of the area pointed by addr */
	uint64_t size;
	/* spare parameter 1 */
	uint64_t param1;
	/* spare parameter 2 */
	uint64_t param2;
	/* private parameter to be used by the guest */
	uint64_t priv_guest;
}  __attribute__((packed)) p4bus_operation_t;

/**
 * p4bus device structure
 */
typedef struct p4bus_device_info_s {
	/**
	 * Device name
	 */
	char name[P4BUS_NAMELEN];
	/**
	 * Device type
	 */
	char type[P4BUS_NAMELEN];
	/**
	 * Device identifier
	 */
	uint32_t p4bus_devid;
	/**
	 * Device identifier
	 */
	uint32_t type_devid;
	/**
	 * Device interrupt number
	 */
	uint32_t interrupt;
	/**
	 * Device operation mask
	 */
	uint32_t opmask;
	/**
	 * Device operation size
	 */
	uint32_t size;
	/**
	 * align
	 */
	uint32_t padding;
}  __attribute__((packed)) p4bus_device_info_t;

/**
 * @purpose Enumeration of p4bus commands
 *
 * When the guest sends a command there are 4 values in
 * the VMM message:
 * - devid : VMM_DEV_P4BUS
 * - arg[0]: msg[0] p4bus command (p4bus_command_t)
 * - arg[1]: msg[1]
 * - arg[2]: msg[2]
 */
typedef enum p4bus_command_e {
    /**
     *  @purpose
     *    Get the p4bus version and the number of p4bus devices
     *
     *  @param msg[0] P4BUS_COMMAND_INFO_VERSION
     *  @param msg[1] not used
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   p4bus API Version
     *  @retval msg[1]:   Number of devices on the p4bus
     *  @retval msg[2]:   unused
     */
    P4BUS_COMMAND_INFO_VERSION = 0,
    /**
     *  @purpose
     *    Return informations on a p4bus device (for a given device ID)
     *
     *  @param msg[0] P4BUS_COMMAND_INFO_DEVICE
     *  @param msg[1] physical address where to store the device info (size of p4bus_device_info_t)
     *  @param msg[2] p4bus device ID
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   physical address where is stored the device info (size of p4bus_device_info_t)
     *  @retval msg[2]:   not used
     */
    P4BUS_COMMAND_INFO_DEVICE = 1,
    /**
     *  @purpose
     *    Create an IORING. This call will inform the manager to attach an
     *    operation thread to the given IORING.
     *    The ioring must be initialized by the guest before.
     *
     *  @param msg[0] P4BUS_COMMAND_CREATE_IORING
     *  @param msg[1] physicall address of the IORING
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   ID of the created IORING
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
    P4BUS_COMMAND_CREATE_IORING = 2,
    /**
     *  @purpose
     *    Destroy an IORING
     *    All operations ongoing are stopped on the host (no signaling)
     *
     *  @param msg[0] P4BUS_COMMAND_DESTROY_IORING
     *  @param msg[1] IORING ID
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
    P4BUS_COMMAND_DESTROY_IORING = 3,
    /**
     *  @purpose
     *    Signal the manager that that the given IORING
     *    has some operations to process.
     *
     *  @param msg[0] P4BUS_COMMAND_SIGNAL_IORING
     *  @param msg[1] IORING ID
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
    P4BUS_COMMAND_SIGNAL_IORING = 4,
    /**
     *  @purpose
     *    Execute a single operation
     *    All operations are done using the same thread
     *
     *  @param msg[0] P4BUS_COMMAND_EXECUTE_OPERATION
     *  @param msg[1] physical address of the operation (p4bus_operation_t)
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   operation ID (to be used for cancel)
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
    P4BUS_COMMAND_EXECUTE_OPERATION = 5,
    /**
     *  @purpose
     *    Cancel a running operation (started with execute)
     *
     *  @param msg[0] P4BUS_COMMAND_CANCEL_OPERATION
     *  @param msg[1] ID of the operation to cancel
     *  @param msg[2] not used
     * 
     *  @returns
     *
     *  @retval msg[0]:   not used
     *  @retval msg[1]:   not used
     *  @retval msg[2]:   not used
     */
	P4BUS_COMMAND_CANCEL_OPERATION = 6,
} p4bus_command_t;


/**
 * @purpose Available operation on a p4bus device
 *
 *  @detailed
 *    Each operation except PING will an end
 *    operation signal. Any non blocking concept
 *    depends on non blocking behaviour on PikeOS side
 *    but will still generate a signal in linux.
 *
 *    Depending on the other side support, some operations
 *    may be unsupported (retcode: P4BUS_E_UNSUPP) or
 *    can simply be emulated (for OPEN or CLOSE for example).
 */

typedef enum p4bus_op_id_e {
	/**
	 * Check that device exists
	 * param1: unused
	 * param2: unused
	 * addr: unused (must be NULL)
	 * size: unused (must be 0)
	 *
	 */
	P4BUS_OP_PING = 0,
	/**
	 * Open
	 * param1: open flags
	 * param2: unused
	 * addr: unused (must be NULL)
	 * size: unused (must be 0)
	 *
	 */
	P4BUS_OP_OPEN = 1,
	/**
	 * Close
	 * param1: unused
	 * param2: unused
	 * addr: unused (must be NULL)
	 * size: unused (must be 0)
	 *
	 */
	P4BUS_OP_CLOSE = 2,
	/**
	 * Cancel an operation
	 * param1: operation ID to cancel
	 * param2: unused
	 * addr: unused (must be NULL)
	 * size: unused (must be 0)
	 *
	 */
	P4BUS_OP_CANCEL = 3,
	/**
	 * Read
	 * param1: offset
	 * param2: if not 0, read at offset in param1
	 * addr: virtual address of buffer
	 * size: buffer size / size read
	 *
	 */
	P4BUS_OP_READ = 4,
	/**
	 * Read non blocking
	 * param1: offset
	 * param2: if not 0, read at offset in param1
	 * addr: virtual address of buffer
	 * size: buffer size / size read
	 *
	 */
	P4BUS_OP_NB_READ = 5,
	/**
	 * Write
	 * param1: offset
	 * param2: if not 0, write at offset in param1
	 * addr: virtual address of buffer
	 * size: buffer size / size written
	 *
	 */
	P4BUS_OP_WRITE = 6,
	/**
	 * Write non blocking
	 * param1: offset
	 * param2: if not 0, write at offset in param1
	 * addr: virtual address of buffer
	 * size: buffer size / size written
	 *
	 */
	P4BUS_OP_NB_WRITE = 7,
	/**
	 * Ioctl
	 * param1: command
	 * param2: size_in (on the most significant 16 bits) | size_out (on the least significant 16 bits)
	 * addr: data virtual address
	 * size: data size
	 *
	 */
	P4BUS_OP_IOCTL = 8,

	/**
	 * Mmap
	 * param1: offset
	 * param2: flags
	 * addr: unused (must be NULL)/physical address
	 * size: size to map
	 *
	 */
	P4BUS_OP_MMAP = 9,
	
	/**
	 * Stat
	 * param1: unused
	 * param2: unused
	 * addr: stat buffer address
	 * size: stat buffer size / size written
	 *
	 */
	P4BUS_OP_STAT = 10,

	/**
	 * lseek
	 * param1: offset
	 * param2: origin
	 * addr: unused (must be NULL) / new file position
	 * size: unused (must be 0)
	 *
	 */
	P4BUS_OP_LSEEK = 11,
	/**
	 * Rest of operation numbers are
	 * custom and can be used for special
	 * purpose.
	 * Operations are used as bit mask values
	 * in a 32 bit so max value is 31.
	 */
	P4BUS_OP_NUM = 32,
} p4bus_op_id_t;

/**
 * @purpose Operation Errors
 *
 *  @detailed
 *    Any operation returns an error
 *    of the following type.
 */
typedef enum p4bus_op_error_e {
	/**
	 * Operation done correctly
	 */
	P4BUS_E_OK = 0x0,
	/**
	 * Operation not supported by driver
	 */
	P4BUS_E_UNSUPP = 0x1,
	/**
	 * Invalid operation argument
	 */
	P4BUS_E_INVAL_ARG = 0x2,
	/**
	 * Bus error during transfer
	 */
	P4BUS_E_BUS = 0x3,
	/**
	 * Operation ongoing
	 */
	P4BUS_E_BUSY = 0x4,
	/**
	 * PikeOS Ukernel Error
	 * P4_e_t = error - P4BUS_E_P4
	 */
	P4BUS_E_P4 = 0x20000,
} p4bus_op_error_t;

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

/* ----------------------- FUNCTION DECLARATIONS --------------------------- */

#endif
