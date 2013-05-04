/*
 * Copyright (c) 2012 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 *
 * Under GPLv2
 */

#ifndef __PBL_H__
#define __PBL_H__

extern unsigned long free_mem_ptr;
extern unsigned long free_mem_end_ptr;

void pbl_barebox_uncompress(void *dest, void *compressed_start, unsigned int len);

#endif /* __PBL_H__ */
