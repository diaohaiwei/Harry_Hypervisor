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

#ifndef P4BUS_H
#define P4BUS_H

/* ------------------------- FILE INCLUSION -------------------------------- */

#include "vmm.h"
#include "pikeos-headers/p4bus-def.h"
#include "pikeos-headers/p4bus-ioring.h"
#include <linux/device.h>

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

/* ----------------------- FUNCTION DECLARATIONS --------------------------- */

int p4bus_detected(void);

/**
 *  @purpose
 *    retrieve the p4bus protocol version and the 
 *    number of devices registered on it.
 *
 *  @param version  pointer on an integer to retrieve the version
 *  @param version  pointer on an integer to retrieve the device number
 *
 *  @returns
 *  @retval P4BUS_E_OK(0)   if no error
 *  @retval -EINVAL         if the call failled
 */
extern int p4bus_command_version(int* version, int* num_devices);

extern int p4bus_command_create_ioring(p4bus_ioring_t* ioring, int* id);

extern int p4bus_command_destroy_ioring(int id);

extern int p4bus_command_signal_ioring(int id);

extern int p4bus_get_devinfo(struct platform_device *pdev, p4bus_device_info_t* device_info);

extern int p4bus_command_execute_operation(p4bus_operation_t *op, int *id);

extern int p4bus_command_cancel_operation(int id);

#endif
