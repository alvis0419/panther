/* 
 *  Copyright (c) 2008, 2009    Acrospeed Inc.    All rights reserved.
 *
 *  bootprom interface
 */
#ifndef _CHEETAH_PROM_H
#define _CHEETAH_PROM_H

extern char *prom_getcmdline(void);
extern char *prom_getenv(char *name);
extern void prom_init_cmdline(void);
extern void prom_meminit(void);

#endif /* !(_CHEETAH_PROM_H) */

