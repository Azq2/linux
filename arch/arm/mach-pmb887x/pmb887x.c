// SPDX-License-Identifier: GPL-2.0-or-later
#include <linux/of.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#define PMB887X_IO_BASE		0xF0000000
#define PMB887X_IO_SIZE		0x0E000000

/* This is needed for LL-debug/earlyprintk/debug-macro.S */
static struct map_desc pmb887x_io_desc[] __initdata = {
	{
		.virtual	= PMB887X_IO_BASE,
		.pfn		= __phys_to_pfn(PMB887X_IO_BASE),
		.length		= PMB887X_IO_SIZE,
		.type		= MT_DEVICE,
	}
};

static void __init pmb887x_init(void) {
	early_printk("pmb887x_init\n");
	of_platform_default_populate(NULL, NULL, NULL);
}

static void __init pmb887x_map_io(void) {
	iotable_init(pmb887x_io_desc, ARRAY_SIZE(pmb887x_io_desc));
}

static const char *pmb887x_board_compat[] = {
	"infineon,pmb887x",
	NULL,
};

DT_MACHINE_START(pmb887x_dt, "Infineon PMB887X")
	.init_machine	= pmb887x_init,
	.map_io			= pmb887x_map_io,
	.dt_compat      = pmb887x_board_compat,
MACHINE_END
