/*
 * KeyASIC KA2000 Power and Sleep Controller (PSC)
 *
 * Copyright (C) 2010 KeyASIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/psc.h>
#include <mach/mux.h>

/* PSC register offsets */
#define EPCPR		0x070
#define PTCMD		0x120
#define PTSTAT		0x128
#define PDSTAT		0x200
#define PDCTL1		0x304
#define MDSTAT		0x800
#define MDCTL		0xA00

/* System control register offsets */
#define VDD3P3V_PWDN	0x48

static void ka2000_psc_mux(unsigned int id)
{
#if 0
	switch (id) {
	case KA2000_LPSC_ATA:
		ka2000_mux_peripheral(KA2000_MUX_HDIREN, 1);
		ka2000_mux_peripheral(KA2000_MUX_ATAEN, 1);
		break;
	case KA2000_LPSC_MMC_SD:
		/* VDD power manupulations are done in U-Boot for CPMAC
		 * so applies to MMC as well
		 */
		/*Set up the pull regiter for MMC */
		ka2000_writel(0, KA2000_SYSTEM_MODULE_BASE + VDD3P3V_PWDN);
		ka2000_mux_peripheral(KA2000_MUX_MSTK, 0);
		break;
	case KA2000_LPSC_I2C:
		ka2000_mux_peripheral(KA2000_MUX_I2C, 1);
		break;
	case KA2000_LPSC_McBSP:
		ka2000_mux_peripheral(KA2000_MUX_ASP, 1);
		break;
	default:
		break;
	}
#endif	
}

/* Enable or disable a PSC domain */
void ka2000_psc_config(unsigned int domain, unsigned int id, char enable)
{
#if 0
	u32 epcpr, ptcmd, ptstat, pdstat, pdctl1, mdstat, mdctl, mdstat_mask;

	mdctl = ka2000_readl(KA2000_PWR_SLEEP_CNTRL_BASE + MDCTL + 4 * id);
	if (enable)
		mdctl |= 0x00000003;	/* Enable Module */
	else
		mdctl &= 0xFFFFFFF2;	/* Disable Module */
	ka2000_writel(mdctl, KA2000_PWR_SLEEP_CNTRL_BASE + MDCTL + 4 * id);

	pdstat = ka2000_readl(KA2000_PWR_SLEEP_CNTRL_BASE + PDSTAT);
	if ((pdstat & 0x00000001) == 0) {
		pdctl1 = ka2000_readl(KA2000_PWR_SLEEP_CNTRL_BASE + PDCTL1);
		pdctl1 |= 0x1;
		ka2000_writel(pdctl1, KA2000_PWR_SLEEP_CNTRL_BASE + PDCTL1);

		ptcmd = 1 << domain;
		ka2000_writel(ptcmd, KA2000_PWR_SLEEP_CNTRL_BASE + PTCMD);

		do {
			epcpr = ka2000_readl(KA2000_PWR_SLEEP_CNTRL_BASE +
					      EPCPR);
		} while ((((epcpr >> domain) & 1) == 0));

		pdctl1 = ka2000_readl(KA2000_PWR_SLEEP_CNTRL_BASE + PDCTL1);
		pdctl1 |= 0x100;
		ka2000_writel(pdctl1, KA2000_PWR_SLEEP_CNTRL_BASE + PDCTL1);

		do {
			ptstat = ka2000_readl(KA2000_PWR_SLEEP_CNTRL_BASE +
					       PTSTAT);
		} while (!(((ptstat >> domain) & 1) == 0));
	} else {
		ptcmd = 1 << domain;
		ka2000_writel(ptcmd, KA2000_PWR_SLEEP_CNTRL_BASE + PTCMD);

		do {
			ptstat = ka2000_readl(KA2000_PWR_SLEEP_CNTRL_BASE +
					       PTSTAT);
		} while (!(((ptstat >> domain) & 1) == 0));
	}

	if (enable)
		mdstat_mask = 0x3;
	else
		mdstat_mask = 0x2;

	do {
		mdstat = ka2000_readl(KA2000_PWR_SLEEP_CNTRL_BASE +
				       MDSTAT + 4 * id);
	} while (!((mdstat & 0x0000001F) == mdstat_mask));

	if (enable)
		ka2000_psc_mux(id);
#endif	
}

void __init ka2000_psc_init(void)
{
//this is mark by kathy
//	ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, KA2000_LPSC_VPSSMSTR, 1);
//	ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, KA2000_LPSC_VPSSSLV, 1);
//	ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, KA2000_LPSC_TPCC, 1);
//	ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, KA2000_LPSC_TPTC0, 1);
//	ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, KA2000_LPSC_TPTC1, 1);
//	ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, KA2000_LPSC_GPIO, 1);
//
//	/* Turn on WatchDog timer LPSC.	 Needed for RESET to work */
//	ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, KA2000_LPSC_TIMER2, 1);
}
