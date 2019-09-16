#ifndef DRV8323_H_
#define DRV8323_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Info
// ----------------------------------------------------------------------
/*
 *
 * Filename:	DRV8323.h
 *
 * Author: 		(c) 2019, Benjamin Prescher
 *
 * Target: 		XXX
 *
 * Notes:
 *
 */
// ----------------------------------------------------------------------
// History
// ----------------------------------------------------------------------
// 09.05.2019 - initial programming

// ----------------------------------------------------------------------
// Header Files
// ----------------------------------------------------------------------
#include "DRV8323_defs.h"
#include "DRV8323_SPI.h"

// ----------------------------------------------------------------------
// Defines
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------
extern DRV8323_VARS_t gDrv8323;

// ----------------------------------------------------------------------
// Functions
// ----------------------------------------------------------------------
void DRV8323_initRegs(DRV8323_VARS_t *v);
void DRV8323_init(DRV8323_VARS_t *v);

// ----------------------------------------------------------------------
// something...
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// End of file
// ----------------------------------------------------------------------


#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /* DRV8323_H_ */