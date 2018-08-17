
EXTRA_FLAGS = -Wall -Werror
EXTRA_FLAGS += $(call cc-disable-warning, misleading-indentation)

ifeq ($(VMM_BUILD_ALL),y)
CONFIG_PIKEOS_VMM := m
CONFIG_VMM_EARLYCON := m
CONFIG_VMM_BYPASS := m
CONFIG_VMM_VMAPI := m
CONFIG_VMM_P4BUS := m
CONFIG_P4BUS_VMCHAR := m
ifeq ($(CONFIG_SERIAL_CORE),y)
CONFIG_P4BUS_VMTTY := m
endif
ifeq ($(CONFIG_NET),y)
CONFIG_P4BUS_VMNET := m
endif
ifeq ($(CONFIG_BLOCK),y)
CONFIG_P4BUS_VMBLOCK := m
endif
endif

has_no_psci := $(shell \
	if [ $(VERSION) -eq 3 \
		-a \$(PATCHLEVEL) -lt 16 ] ; then \
		echo "true" ; fi;)

obj-$(CONFIG_PIKEOS_VMM)					+= vmm.o
obj-$(CONFIG_VMM_EARLYCON)					+= vmm-earlycon.o
obj-$(CONFIG_VMM_BYPASS)					+= vmm-bypass.o
obj-$(CONFIG_VMM_VMAPI)						+= vmm-vmapi.o
obj-$(CONFIG_VMM_P4BUS)						+= p4bus.o
ifeq ($(CONFIG_ARM), y)
ifeq ($(has_no_psci),true)
obj-y										+= vmm-psci.o
endif
endif

obj-$(CONFIG_P4BUS_VMCHAR)					+= p4bus-vmchar.o
obj-$(CONFIG_P4BUS_VMTTY)					+= p4bus-vmtty.o
obj-$(CONFIG_P4BUS_VMNET)					+= p4bus-vmnet.o
obj-$(CONFIG_P4BUS_VMBLOCK)					+= p4bus-vmblock.o

ifeq ($(CONFIG_ARM), y)
has_virt := $(call as-instr,.arch_extension virt,true)
ifneq ($(has_virt),true)
EXTRA_FLAGS += -DNO_VIRT_EXT
endif
endif


CFLAGS_vmm.o += $(EXTRA_FLAGS)
CFLAGS_vmm-earlycon.o += $(EXTRA_FLAGS)
CFLAGS_vmm-bypass.o += $(EXTRA_FLAGS)
CFLAGS_vmm-vmapi.o += $(EXTRA_FLAGS)
CFLAGS_p4bus.o += $(EXTRA_FLAGS)
ifeq ($(has_no_psci),true)
CFLAGS_vmm-psci.o += $(EXTRA_FLAGS)
endif

CFLAGS_p4bus-vmchar.o += $(EXTRA_FLAGS)
CFLAGS_p4bus-vmtty.o += $(EXTRA_FLAGS)
CFLAGS_p4bus-vmnet.o += $(EXTRA_FLAGS)
CFLAGS_p4bus-vmblock.o += $(EXTRA_FLAGS)
