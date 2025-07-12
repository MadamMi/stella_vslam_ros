#ifndef __PLNSTANDBY_H
#define __PLNSTANDBY_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

int8_t plannerStandbyEnter(void);
int8_t plannerStandbyRun(void);
int8_t plannerStandbyExit(void);
	 
#ifdef __cplusplus
}
#endif

#endif

