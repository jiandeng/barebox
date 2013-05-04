/*
 * Copyright (C) 2012 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 */

#ifndef __HW_REVISION_H__
#define __HW_REVISION_H__

enum vendor_id {
	VENDOR_UNKNOWN	= 0,
	VENDOR_EMBEST	= 1,
	VENDOR_FLEX	= 2,
	VENDOR_RONETIX	= 3,
	VENDOR_COGENT	= 4,
	VENDOR_PDA	= 5,
};

#ifdef CONFIG_W1
bool at91sama5d3xek_cm_is_vendor(enum vendor_id vid);
bool at91sama5d3xek_ek_is_vendor(enum vendor_id vid);
bool at91sama5d3xek_dm_is_vendor(enum vendor_id vid);
void at91sama5d3xek_devices_detect_hw(void);
#else
bool at91sama5d3xek_cm_is_vendor(enum vendor_id vid)
{
	return false;
}

bool at91sama5d3xek_ek_is_vendor(enum vendor_id vid)
{
	return false;
}

bool at91sama5d3xek_dm_is_vendor(enum vendor_id vid)
{
	return false;
}

void at91sama5d3xek_devices_detect_hw(void) {}
#endif

#endif /* __HW_REVISION_H__ */
