/*
 * This header provides constants for the ARM GIC.
 */

#ifndef _DT_BINDINGS_INTERRUPT_CONTROLLER_ARM_GIC_H
#define _DT_BINDINGS_INTERRUPT_CONTROLLER_ARM_GIC_mZ

/*Pikeos device tree virtualization support by Harry*/

#define PIKEOS_GUEST
/*#define PIKEOS_GUEST_2*/
#define PIKEOS_GUEST_PCI
#define PIKEOS_GUEST_SD
#define PIKEOS_GUEST_GPU
#define PIKEOS_GUEST_USB
#define PIKEOS_GUEST_EMMC

#include <dt-bindings/interrupt-controller/irq.h>

/* interrupt specifier cell 0 */

#define GIC_SPI 0
#define GIC_PPI 1

/*
 * Interrupt specifier cell 2.
 * The flags in irq.h are valid, plus those below.
 */
#define GIC_CPU_MASK_RAW(x) ((x) << 8)
#define GIC_CPU_MASK_SIMPLE(num) GIC_CPU_MASK_RAW((1 << (num)) - 1)

#endif
