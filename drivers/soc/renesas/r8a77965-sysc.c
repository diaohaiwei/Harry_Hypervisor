/*
 * Renesas R-Car M3N System Controller
 *
 * Copyright (C) 2017 Renesas Electronics Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/bug.h>
#include <linux/kernel.h>

#include <dt-bindings/power/r8a77965-sysc.h>

#include "rcar-sysc.h"

static const struct rcar_sysc_area r8a77965_areas[] __initconst = {
	{ "always-on",	    0, 0, R8A77965_PD_ALWAYS_ON, -1, PD_ALWAYS_ON },
	{ "ca57-scu",	0x1c0, 0, R8A77965_PD_CA57_SCU,  R8A77965_PD_ALWAYS_ON,
	  PD_SCU },
	{ "ca57-cpu0",	 0x80, 0, R8A77965_PD_CA57_CPU0, R8A77965_PD_CA57_SCU,
	  PD_CPU_NOCR },
	{ "ca57-cpu1",	 0x80, 1, R8A77965_PD_CA57_CPU1, R8A77965_PD_CA57_SCU,
	  PD_CPU_NOCR },
	{ "cr7",	0x240, 0, R8A77965_PD_CR7,	R8A77965_PD_ALWAYS_ON,
	  (PD_NO_CR | PD_ON_ONCE) },
	{ "a3vp",	0x340, 0, R8A77965_PD_A3VP,	R8A77965_PD_ALWAYS_ON,
	  (PD_NO_CR | PD_ON_ONCE) },
	{ "a3vc",	0x380, 0, R8A77965_PD_A3VC,	R8A77965_PD_ALWAYS_ON,
	  (PD_NO_CR | PD_ON_ONCE) },
	{ "a2vc1",	0x3c0, 1, R8A77965_PD_A2VC1,	R8A77965_PD_A3VC,
	  (PD_NO_CR | PD_ON_ONCE) },
	{ "3dg-a",	0x100, 0, R8A77965_PD_3DG_A,	R8A77965_PD_ALWAYS_ON },
	{ "3dg-b",	0x100, 1, R8A77965_PD_3DG_B,	R8A77965_PD_3DG_A },
	{ "a3ir",	0x180, 0, R8A77965_PD_A3IR,	R8A77965_PD_ALWAYS_ON,
	  (PD_NO_CR | PD_ON_ONCE) },
};

const struct rcar_sysc_info r8a77965_sysc_info __initconst = {
	.areas = r8a77965_areas,
	.num_areas = ARRAY_SIZE(r8a77965_areas),
};
