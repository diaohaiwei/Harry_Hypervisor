#ifndef COMPAT_H
#define COMPAT_H
/**
 * This headers provide some compatibility defines
 * to provide functions for older kernels
 */
#include <linux/version.h>

#ifndef __ARM_ARCH
#ifdef CONFIG_ARM
#ifdef CONFIG_CPU_32v7
#define __ARM_ARCH 7
#endif
#elif defined(CONFIG_ARM64)
#define __ARM_ARCH 8
#else
#error Unsupported architecture
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
#define module_platform_driver(__platform_driver) \
static int __init __platform_driver##_init(void) \
{ \
        return platform_driver_register(&(__platform_driver)); \
} \
module_init(__platform_driver##_init); \
static void __exit __platform_driver##_exit(void) \
{ \
       platform_driver_unregister(&(__platform_driver)); \
} \
module_exit(__platform_driver##_exit);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#define PDE_DATA(node)	PDE(node)->data
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
#define smp_set_ops(op)		do {} while (0)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
#define of_platform_populate(a, b, c, d)	of_platform_bus_probe(a, b, d)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
#define file_inode(f)	(file->f_path.dentry->d_inode)
#endif
#endif
