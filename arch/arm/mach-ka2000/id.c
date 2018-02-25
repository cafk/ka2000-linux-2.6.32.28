/*
 * KeyASIC Ka2000 CPU identification code
 *
 * Copyright (C) 2013 KeyASIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#define JTAG_ID_BASE		0x01c40028

struct ka2000_id {
	u8	variant;	/* JTAG ID bits 31:28 */
	u16	part_no;	/* JTAG ID bits 27:12 */
	u32	manufacturer;	/* JTAG ID bits 11:1 */
	u32	type;		/* Cpu id bits [31:8], cpu class bits [7:0] */
};

/* Register values to detect the KeyASIC Ka2000 version */
static struct ka2000_id ka2000_ids[] __initdata = {
	{
		/* DM6446 */
		.part_no      = 0xb700,
		.variant      = 0x0,
		.manufacturer = 0x017,
		.type	      = 0x64460000,
	},
};

/*
 * Get Device Part No. from JTAG ID register
 */
static u16 __init ka2000_get_part_no(void)
{
	u32 dev_id, part_no;

	dev_id = ka2000_readl(JTAG_ID_BASE);

	part_no = ((dev_id >> 12) & 0xffff);

	return part_no;
}

/*
 * Get Device Revision from JTAG ID register
 */
static u8 __init ka2000_get_variant(void)
{
	u32 variant;

	variant = ka2000_readl(JTAG_ID_BASE);

	variant = (variant >> 28) & 0xf;

	return variant;
}

void __init ka2000_check_revision(void)
{
	int i;
	u16 part_no;
	u8 variant;

	part_no = ka2000_get_part_no();
	variant = ka2000_get_variant();

	/* First check only the major version in a safe way */
	for (i = 0; i < ARRAY_SIZE(ka2000_ids); i++) {
		if (part_no == (ka2000_ids[i].part_no)) {
			system_rev = ka2000_ids[i].type;
			break;
		}
	}

	/* Check if we can find the dev revision */
	for (i = 0; i < ARRAY_SIZE(ka2000_ids); i++) {
		if (part_no == ka2000_ids[i].part_no &&
		    variant == ka2000_ids[i].variant) {
			system_rev = ka2000_ids[i].type;
			break;
		}
	}

	printk("KeyASIC Ka2000 DM%04x variant 0x%x\n", system_rev >> 16, variant);
}
