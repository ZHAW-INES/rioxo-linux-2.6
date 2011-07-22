/*
 * Copyright (C) RuggedCom 2010
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 * Description:
 * Adapted from isp1760.h
 *
 * board initialization should put one of these into dev->platform_data
 * and place the isp1763 onto platform_bus named "isp1763-hcd".
 */

#ifndef __LINUX_USB_ISP1763_H
#define __LINUX_USB_ISP1763_H

struct isp1763_platform_data {
	unsigned bus_width_8:1;		/* 8/16-bit data bus width */
	unsigned port1_otg:2;		/* Port 1 supports OTG */
	unsigned dack_polarity_high:1;	/* DACK active high */
	unsigned dreq_polarity_high:1;	/* DREQ active high */
	unsigned intr_polarity_high:1;	/* INTR active high */
	unsigned intr_edge_trigger:1;	/* INTR edge trigger */
};

#endif /* __LINUX_USB_ISP1763_H */
