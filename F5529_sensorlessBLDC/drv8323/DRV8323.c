// ----------------------------------------------------------------------
// Info
// ----------------------------------------------------------------------
/*
 *
 * Filename:	DRV8323.c
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
#include "driverlib.h"
#include "delay.h"
#include "DRV8323.h"

// ----------------------------------------------------------------------
// Defines
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------
DRV8323_VARS_t gDrv8323 = DRV8323_DEFAULTS;

// ----------------------------------------------------------------------
// Functions
// ----------------------------------------------------------------------
void DRV8323_initRegs(DRV8323_VARS_t *v)
{
    // Set Register Driver Control Register (0x02)
    v->Driver_Control.bit.DIS_CPUV    = drv_chargePump_faultEnable;
    v->Driver_Control.bit.DIS_GDF     = drv_gateDrive_faultEnable;
    v->Driver_Control.bit.OTW_REP     = drv_otw_no_report;
    v->Driver_Control.bit.PWM_MODE    = drv_PWM_mode_1;
    v->Driver_Control.bit.PWM_COM_1   = drv_1xPWM_synch;
    v->Driver_Control.bit.PWM_DIR_1   = drv_1xPWM_noChangeDir;
    v->Driver_Control.bit.COAST       = drv_coastDisable;
    v->Driver_Control.bit.BRAKE       = drv_breakDisable;
    v->Driver_Control.bit.CLR_FLT     = drv_leaveFaults;

    // Set Register Gate Drive HS Register (0x03)
    v->Gate_Drive_HS.bit._LOCK        = drv_unlock;
    v->Gate_Drive_HS.bit.IDRIVEP_HS   = drv_idriveP_hs_1000mA;
    v->Gate_Drive_HS.bit.IDRIVEN_HS   = drv_idriveN_hs_2000mA;

    // Set Register Gate Drive LS Register (0x04)
    v->Gate_Drive_LS.bit.CBC          = drv_clrFaults;
    v->Gate_Drive_LS.bit.TDRIVE       = drv_tdrive_4000nS;
    v->Gate_Drive_LS.bit.IDRIVEP_LS   = drv_idriveP_ls_1000mA;
    v->Gate_Drive_LS.bit.IDRIVEN_LS   = drv_idriveN_ls_2000mA;

    // Set Register OCP Control Register (0x05)
    v->OCP_Control.bit.TRETRY         = drv_retry_time_4ms;
    v->OCP_Control.bit.DEAD_TIME      = drv_deadTime_100nS;
    v->OCP_Control.bit.OCP_MODE       = drv_ocp_mode_autoRetryFault;
    v->OCP_Control.bit.OCP_DEG        = drv_ocp_deg_4us;
    v->OCP_Control.bit.VDS_LVL        = drv_vds_lvl_750mV;

    // Set Register CSA Control Register (0x06)
    v->CSA_Control.bit.CSA_FET        = drv_input_SPx;
    v->CSA_Control.bit.VREF_DIV       = drv_vref_div_2;   // bidirectional
    v->CSA_Control.bit.LS_REF         = drv_vds_ocp_SHx_SPx;
    v->CSA_Control.bit.CSA_GAIN       = drv_gain_20;

    v->CSA_Control.bit.DIS_SEN        = drv_oc_faultEnable;
    v->CSA_Control.bit.CSA_CAL_A      = drv_normal_sense;
    v->CSA_Control.bit.CSA_CAL_B      = drv_normal_sense;
    v->CSA_Control.bit.CSA_CAL_C      = drv_normal_sense;
    v->CSA_Control.bit.SEN_LVL        = drv_ocp_1000mV;
}

// ----------------------------------------------------------------------
//
void DRV8323_init(DRV8323_VARS_t *v)
{
    DRV8323_SPI_Write(v, DRV8323_DRIVER_CNTRL_ADDR);         //write to DRV8323
    DRV8323_SPI_Write(v, DRV8323_GATE_DRIVE_HS_ADDR);        //write to DRV8323
    DRV8323_SPI_Write(v, DRV8323_GATE_DRIVE_LS_ADDR);        //write to DRV8323
    DRV8323_SPI_Write(v, DRV8323_OCP_CNTRL_ADDR);            //write to DRV8323
    DRV8323_SPI_Write(v, DRV8323_CSA_CNTRL_ADDR);            //write to DRV8323

    uint16_t i;
    for(i= 0; i < DRV8323_CSA_CNTRL_ADDR + 1; i++)
    {
        DRV8323_SPI_Read(v, i);
        delay_us(5);
    }
}
// ----------------------------------------------------------------------
// something...
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// End of file
// ----------------------------------------------------------------------

