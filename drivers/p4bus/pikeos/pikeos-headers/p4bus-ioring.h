/*
 *  PikeOS P4Bus iorings protocol definition
 *
 *  Copyright (C) 2014, SYSGO AG
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */

#ifndef P4BUS_IORING_H_
#define P4BUS_IORING_H_

/* ------------------------- FILE INCLUSION -------------------------------- */

#include "p4bus-def.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

/**
 * IORING entry structure
 * should be a power of 2
 */
#define P4BUS_IORING_DEPTH  64

/** IORING MASK */
#define P4BUS_IORING_OP_MASK  ((P4BUS_IORING_DEPTH) - 1)

#ifndef NULL
#define NULL	((void *)0)
#endif

#define ioring_barrier()	__asm__ __volatile("": : :"memory")

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

/**
 * IORING structure
 *
 * curr_ready is handled locally by destination
 * as pointer to the relevant functions.
 *
 */
typedef struct p4bus_ioring_s {
	/* index of the FREE entry in the ioring */
	uint32_t curr_free;
	/* some padding for 64bit systems */
	uint32_t unused1;
	/* index of the DONE entry in the ioring */
	uint32_t curr_done;
	/* some padding for 64bit systems */
	uint32_t unused2;
	/* p4bus operations IORING */
	p4bus_operation_t operations[P4BUS_IORING_DEPTH];
}  __attribute__((packed)) p4bus_ioring_t;

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

/* ----------------------- FUNCTION DECLARATIONS --------------------------- */

/**
 *  @purpose
 *    Initialize an ioring.
 *    This function must be called the P4BUS command "P4BUS_COMMAND_CREATE_IORING"
 *    on the targeted IORING.
 *
 *  @param *ioring: pointer on the ioring structure to initialize
 * 
 */
static inline void p4bus_ioring_init(p4bus_ioring_t* ioring)
{
	uint32_t i = 0;
	ioring->curr_free=0;
	ioring->curr_done=0;

	for (i = 0; i<P4BUS_IORING_DEPTH; i++) {
		ioring->operations[i].status = P4BUS_OPERATION_FREE;
	}

}

/**
 *  @purpose
 *    Check if a free operation is available
 *
 *  @param *ioring: pointer on the ioring targeted
 * 
 *  @returns
 *
 *  @retval TRUE(0x1) :   if a free operation is available
 *  @retval FALSE(0x0):   if there is no free operation
 */
static inline uint32_t p4bus_ioring_has_free(p4bus_ioring_t* ioring)
{
	return (ioring->operations[ioring->curr_free].status == P4BUS_OPERATION_FREE);
}

/**
 *  @purpose
 *    Retrieve a free operation.
 *    This function must be called in a critical section.
 *
 *  @param *ioring: pointer on the ioring targeted
 * 
 *  @returns
 *      return the address of the first free operation available
 *
 *  @retval NULL :   if no free operation is available
 */
static inline p4bus_operation_t* p4bus_ioring_get_free(p4bus_ioring_t* ioring)
{
	p4bus_operation_t* tmp;
	tmp = &(ioring->operations[ioring->curr_free]);
	if(tmp->status == P4BUS_OPERATION_FREE)
	{
		ioring->curr_free = (ioring->curr_free + 1) & P4BUS_IORING_OP_MASK;
		return tmp;
	}
	return NULL;
}

/**
 *  @purpose
 *    Push a free operation in the IORING
 *
 *  @param *ioring_entry: pointer on the ioring operation to push as free
 * 
 */
static inline void p4bus_ioring_push_free(p4bus_operation_t* ioring_entry)
{
	ioring_barrier();
	ioring_entry->status = P4BUS_OPERATION_FREE;
}

/**
 *  @purpose
 *    Check if a ready operation is available
 *
 *  @param *ioring: pointer on the ioring targeted
 * 
 *  @returns
 *
 *  @retval TRUE(0x1) :   if a ready operation is available
 *  @retval FALSE(0x0):   if there is no ready operation
 */
static inline uint32_t p4bus_ioring_has_ready(p4bus_ioring_t* ioring, uint32_t *curr_ready)
{
	return ((ioring->operations[*curr_ready].status) == P4BUS_OPERATION_READY);
}

/**
 *  @purpose
 *    Retrieve ready-to-process operation 
 *    This function must be called in a critical section.
 *
 *  @param *ioring: pointer on the ioring targeted
 * 
 *  @returns
 *      return the address of the first ready operation available
 *
 *  @retval NULL :   if no ready operation is available
 */
static inline p4bus_operation_t* p4bus_ioring_get_ready(p4bus_ioring_t* ioring, uint32_t *curr_ready)
{
	p4bus_operation_t* tmp;
	tmp = &(ioring->operations[*curr_ready]);
	if(tmp->status == P4BUS_OPERATION_READY)
	{
		*curr_ready = (*curr_ready + 1) & P4BUS_IORING_OP_MASK;
		return tmp;
	}
	return NULL;
}

/**
 *  @purpose
 *    Push a ready-to-process operation
 *
 *  @param *ioring_entry: pointer on the ioring operation to push as ready
 * 
 */
static inline void p4bus_ioring_push_ready(p4bus_operation_t* ioring_entry)
{
	ioring_barrier();
	ioring_entry->status = P4BUS_OPERATION_READY;
}

/**
 *  @purpose
 *    Check if a done operation is available
 *
 *  @param *ioring: pointer on the ioring targeted
 * 
 *  @returns
 *
 *  @retval TRUE(0x1) :   if a done operation is available
 *  @retval FALSE(0x0):   if there is no done operation
 */
static inline uint32_t p4bus_ioring_has_done(p4bus_ioring_t* ioring)
{
	return ((ioring->operations[ioring->curr_done].status) == P4BUS_OPERATION_DONE);
}

/**
 *  @purpose
 *    Retrieve done operation
 *    This function must be called in a critical section. 
 *
 *  @param *ioring: pointer on the ioring targeted
 * 
 *  @returns
 *      return the address of the first done operation available
 *
 *  @retval NULL :   if no done operation is available
 */
static inline p4bus_operation_t* p4bus_ioring_get_done(p4bus_ioring_t* ioring)
{
	p4bus_operation_t* tmp;
	tmp = &(ioring->operations[(ioring->curr_done)]);
	if(tmp->status == P4BUS_OPERATION_DONE)
	{
		ioring->curr_done = (ioring->curr_done + 1)  & P4BUS_IORING_OP_MASK;
		return tmp;
	}
	return NULL;
}

/**
 *  @purpose
 *    Push a done operation in the IORING
 *
 *  @param *ioring_entry: pointer on the ioring operation to push as done
 * 
 */
static inline void p4bus_ioring_push_done(p4bus_operation_t* ioring_entry)
{
	ioring_barrier();
	ioring_entry->status = P4BUS_OPERATION_DONE;
}

#endif /* P4BUS_IORING_H_ */
