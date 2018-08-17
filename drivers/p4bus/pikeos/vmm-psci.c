/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2012 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 */

/* ------------------------- FILE INCLUSION -------------------------------- */

#define pr_fmt(fmt) "psci: " fmt

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/pm.h>

#include <asm/mach/arch.h>
#include <asm/smp_plat.h>
#include <asm/errno.h>
#include <asm/memory.h>
#include <linux/version.h>
#include <linux/of.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#else
#include <asm/system_misc.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
#include <asm/hardware/gic.h>
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
#include <asm/smp.h>
#endif

#include "compat.h"
#include "vmm-psci.h"


/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF			"VMM-PSCI: "
#define DRIVER_NAME		"vmm-psci"



/* ------------------------ TYPE DECLARATIONS ------------------------------ */

enum vmm_psci_function {
	PSCI_FN_CPU_SUSPEND,
	PSCI_FN_CPU_ON,
	PSCI_FN_CPU_OFF,
	PSCI_FN_MIGRATE,
	PSCI_FN_AFFINITY_INFO,
	PSCI_FN_MIGRATE_INFO_TYPE,
	PSCI_FN_MAX,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
struct smp_operations {
#ifdef CONFIG_SMP
    void (*smp_init_cpus)(void);
    void (*smp_prepare_cpus)(unsigned int max_cpus);
    void (*smp_secondary_init)(unsigned int cpu);
    int  (*smp_boot_secondary)(unsigned int cpu, struct task_struct *idle);
#endif
};
#endif

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

#if __ARM_ARCH == 7
static int (*invoke_psci_fn)(u32, u32, u32, u32);
#else
static int (*invoke_psci_fn)(u64, u64, u64, u64);
#endif

typedef int (*psci_initcall_t)(const struct device_node *);

static int vmm_psci_boot_secondary(unsigned int cpu, struct task_struct *idle);

static int vmm_psci_0_2_init(struct device_node *np);

static void __init vmm_psci_smp_init_cpus(void);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
#ifdef CONFIG_HOTPLUG_CPU
void __ref vmm_psci_cpu_die(unsigned int cpu);

int __ref vmm_psci_cpu_kill(unsigned int cpu);
#endif
#endif


/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

struct psci_operations vmm_psci_ops;

static u32 vmm_psci_function_id[PSCI_FN_MAX];

struct smp_operations __initdata vmm_psci_smp_ops = {
	.smp_init_cpus		= vmm_psci_smp_init_cpus,
	.smp_boot_secondary	= vmm_psci_boot_secondary,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= vmm_psci_cpu_die,
	.cpu_kill		= vmm_psci_cpu_kill,
#endif
#endif
};

static const struct of_device_id vmm_psci_of_match[] __initconst = {
	{ .compatible = "arm,psci-0.2", .data = vmm_psci_0_2_init},
	{},
};

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */

extern void secondary_startup(void);

/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */

static int __init vmm_psci_early_init(void)
{
	struct device_node *np;

	np = of_find_matching_node(NULL, vmm_psci_of_match);

	if (!np)
	{
		pr_err(PREF "Wrong PSCI version found in the DTB\n");
		return -ENODEV;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	((struct machine_desc *)machine_desc)->smp = &vmm_psci_smp_ops;
#endif
#ifdef smp_set_ops
	smp_set_ops(&vmm_psci_smp_ops);
#endif
	
	vmm_psci_0_2_init(np);


	return 0;
}
early_initcall(vmm_psci_early_init);


static int __init vmm_psci_early_param(char *buf)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	((struct machine_desc *)machine_desc)->smp = &vmm_psci_smp_ops;
#endif
#ifdef smp_set_ops
	smp_set_ops(&vmm_psci_smp_ops);
#endif

	return 1;
}
early_param("vmm", vmm_psci_early_param);

static int vmm_psci_to_linux_errno(int errno)
{
	switch (errno) {
	case PSCI_RET_SUCCESS:
		return 0;
	case PSCI_RET_NOT_SUPPORTED:
		return -EOPNOTSUPP;
	case PSCI_RET_INVALID_PARAMS:
		return -EINVAL;
	case PSCI_RET_DENIED:
		return -EPERM;
	};

	return -EINVAL;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init vmm_psci_smp_init_cpus(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
	set_smp_cross_call(gic_raise_softirq);
#endif
}

static u32 vmm_psci_power_state_pack(struct psci_power_state state)
{
	return ((state.id << PSCI_0_2_POWER_STATE_ID_SHIFT)
			& PSCI_0_2_POWER_STATE_ID_MASK) |
		((state.type << PSCI_0_2_POWER_STATE_TYPE_SHIFT)
		 & PSCI_0_2_POWER_STATE_TYPE_MASK) |
		((state.affinity_level << PSCI_0_2_POWER_STATE_AFFL_SHIFT)
		 & PSCI_0_2_POWER_STATE_AFFL_MASK);
}

static int vmm_psci_get_version(void)
{
	int err;

	err = invoke_psci_fn(PSCI_0_2_FN_PSCI_VERSION, 0, 0, 0);
	return err;
}

static int vmm_psci_cpu_suspend(struct psci_power_state state,
			    unsigned long entry_point)
{
	int err;
	u32 fn, power_state;

	fn = vmm_psci_function_id[PSCI_FN_CPU_SUSPEND];
	power_state = vmm_psci_power_state_pack(state);
	err = invoke_psci_fn(fn, power_state, entry_point, 0);
	return vmm_psci_to_linux_errno(err);
}

static int vmm_psci_cpu_off(struct psci_power_state state)
{
	int err;
	u32 fn, power_state;

	fn = vmm_psci_function_id[PSCI_FN_CPU_OFF];
	power_state = vmm_psci_power_state_pack(state);
	err = invoke_psci_fn(fn, power_state, 0, 0);
	return vmm_psci_to_linux_errno(err);
}

static int vmm_psci_cpu_on(unsigned long cpuid, unsigned long entry_point)
{
	int err;
	u32 fn;

	fn = vmm_psci_function_id[PSCI_FN_CPU_ON];
	err = invoke_psci_fn(fn, cpuid, entry_point, 0);
	return vmm_psci_to_linux_errno(err);
}

static int vmm_psci_migrate(unsigned long cpuid)
{
	int err;
	u32 fn;

	fn = vmm_psci_function_id[PSCI_FN_MIGRATE];
	err = invoke_psci_fn(fn, cpuid, 0, 0);
	return vmm_psci_to_linux_errno(err);
}

static int vmm_psci_affinity_info(unsigned long target_affinity,
		unsigned long lowest_affinity_level)
{
	int err;
	u32 fn;

	fn = vmm_psci_function_id[PSCI_FN_AFFINITY_INFO];
	err = invoke_psci_fn(fn, target_affinity, lowest_affinity_level, 0);
	return err;
}

static int vmm_psci_migrate_info_type(void)
{
	int err;
	u32 fn;

	fn = vmm_psci_function_id[PSCI_FN_MIGRATE_INFO_TYPE];
	err = invoke_psci_fn(fn, 0, 0, 0);
	return err;
}

static int get_set_conduit_method(struct device_node *np)
{
	const char *method;

	pr_info("probing for conduit method from DT.\n");

	if (of_property_read_string(np, "method", &method)) {
		pr_warn("missing \"method\" property\n");
		return -ENXIO;
	}

	if (!strcmp("hvc", method)) {
		invoke_psci_fn = vmm_invoke_psci_fn_hvc;
	} else {
		pr_warn("invalid \"method\" property: %s\n", method);
		return -EINVAL;
	}
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
static void vmm_psci_sys_reset(char  str, const char *cmd)
{
	invoke_psci_fn(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0);
}
#else
static void vmm_psci_sys_reset(enum reboot_mode reboot_mode, const char *cmd)
{
	invoke_psci_fn(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0);
}
#endif

static void vmm_psci_sys_poweroff(void)
{
	invoke_psci_fn(PSCI_0_2_FN_SYSTEM_OFF, 0, 0, 0);
}

/*
 * PSCI Function IDs for v0.2+ are well defined so use
 * standard values.
 */
static int vmm_psci_0_2_init(struct device_node *np)
{
	int err, ver;

	err = get_set_conduit_method(np);

	if (err)
		goto out_put_node;

	ver = vmm_psci_get_version();

	if (ver == PSCI_RET_NOT_SUPPORTED) {
		/* PSCI v0.2 mandates implementation of PSCI_ID_VERSION. */
		pr_err("PSCI firmware does not comply with the v0.2 spec.\n");
		err = -EOPNOTSUPP;
		goto out_put_node;
	} else {
		pr_info("PSCIv%d.%d detected in firmware.\n",
				PSCI_VERSION_MAJOR(ver),
				PSCI_VERSION_MINOR(ver));

		if (PSCI_VERSION_MAJOR(ver) == 0 &&
				PSCI_VERSION_MINOR(ver) < 2) {
			err = -EINVAL;
			pr_err("Conflicting PSCI version detected.\n");
			goto out_put_node;
		}
	}

	pr_info("Using standard PSCI v0.2 function IDs\n");
	vmm_psci_function_id[PSCI_FN_CPU_SUSPEND] = PSCI_0_2_FN_CPU_SUSPEND;
	vmm_psci_ops.cpu_suspend = vmm_psci_cpu_suspend;

	vmm_psci_function_id[PSCI_FN_CPU_OFF] = PSCI_0_2_FN_CPU_OFF;
	vmm_psci_ops.cpu_off = vmm_psci_cpu_off;

	vmm_psci_function_id[PSCI_FN_CPU_ON] = PSCI_0_2_FN_CPU_ON;
	vmm_psci_ops.cpu_on = vmm_psci_cpu_on;

	vmm_psci_function_id[PSCI_FN_MIGRATE] = PSCI_0_2_FN_MIGRATE;
	vmm_psci_ops.migrate = vmm_psci_migrate;

	vmm_psci_function_id[PSCI_FN_AFFINITY_INFO] = PSCI_0_2_FN_AFFINITY_INFO;
	vmm_psci_ops.affinity_info = vmm_psci_affinity_info;

	vmm_psci_function_id[PSCI_FN_MIGRATE_INFO_TYPE] =
		PSCI_0_2_FN_MIGRATE_INFO_TYPE;
	vmm_psci_ops.migrate_info_type = vmm_psci_migrate_info_type;

	arm_pm_restart = vmm_psci_sys_reset;

	pm_power_off = vmm_psci_sys_poweroff;

out_put_node:
	of_node_put(np);
	return err;
}

static int vmm_psci_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	if (vmm_psci_ops.cpu_on)
		return vmm_psci_ops.cpu_on(cpu_logical_map(cpu),
				virt_to_phys(secondary_startup));
	return -ENODEV;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
#ifdef CONFIG_HOTPLUG_CPU
void __ref vmm_psci_cpu_die(unsigned int cpu)
{
       const struct psci_power_state ps = {
               .type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
       };

       if (vmm_psci_ops.cpu_off)
               vmm_psci_ops.cpu_off(ps);

       /* We should never return */
       panic("psci: cpu %d failed to shutdown\n", cpu);
}

int __ref vmm_psci_cpu_kill(unsigned int cpu)
{
	int err, i;

	if (!vmm_psci_ops.affinity_info)
		return 1;
	/*
	 * cpu_kill could race with cpu_die and we can
	 * potentially end up declaring this cpu undead
	 * while it is dying. So, try again a few times.
	 */

	for (i = 0; i < 10; i++) {
		err = vmm_psci_ops.affinity_info(cpu_logical_map(cpu), 0);
		if (err == PSCI_0_2_AFFINITY_LEVEL_OFF) {
			pr_info("CPU%d killed.\n", cpu);
			return 1;
		}

		msleep(10);
		pr_info("Retrying again to check for CPU kill\n");
	}

	pr_warn("CPU%d may not have shut down cleanly (AFFINITY_INFO reports %d)\n",
			cpu, err);
	/* Make platform_cpu_kill() fail. */
	return 0;
}

#endif
#endif

bool __init vmm_psci_smp_available(void)
{
	/* is cpu_on available at least? */
	return (vmm_psci_ops.cpu_on != NULL);
}


int __init vmm_psci_init(void)
{
	struct device_node *np;

	np = of_find_matching_node(NULL, vmm_psci_of_match);
	if (!np)
	{
		pr_err(PREF "Wrong PSCI version found in the DTB\n");
		return -ENODEV;
	}


	return vmm_psci_0_2_init(np);
}
