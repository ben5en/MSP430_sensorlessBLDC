#ifndef DRV8323_DEFS_H_
#define DRV8323_DEFS_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Info
// ----------------------------------------------------------------------
/*
 *
 * Filename:	DRV8323_defs.h
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

// ----------------------------------------------------------------------
// Defines
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// Register Addresses
// ----------------------------------------------------------------------
//DRV8323 Register Addresses
#define DRV8323_FAULT_STATUS_1_ADDR             0x00
#define DRV8323_VGS_STATUS_2_ADDR               0x01
#define DRV8323_DRIVER_CNTRL_ADDR               0x02
#define DRV8323_GATE_DRIVE_HS_ADDR              0x03
#define DRV8323_GATE_DRIVE_LS_ADDR              0x04
#define DRV8323_OCP_CNTRL_ADDR                  0x05
#define DRV8323_CSA_CNTRL_ADDR                  0x06

// ----------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// DRV8323 Fault Status Register 1 (address = 0x00)
// ----------------------------------------------------------------------
struct  DRV8323_FAULT_STATUS_1_REG_BITS
{                                  // bit  description
    uint16_t VDS_LC:1;           // 0    overcurrent fault C low-side MOSFET
    uint16_t VDS_HC:1;           // 1    overcurrent fault C high-side MOSFET
    uint16_t VDS_LB:1;           // 2    overcurrent fault B low-side MOSFET
    uint16_t VDS_HB:1;           // 3    overcurrent fault B high-side MOSFET
    uint16_t VDS_LA:1;           // 4    overcurrent fault A low-side MOSFET
    uint16_t VDS_HA:1;           // 5    overcurrent fault A high-side MOSFET
    uint16_t OTSD:1;             // 6    overtemperature shutdown
    uint16_t UVLO:1;             // 7    undervoltage lockout fault
    uint16_t GDF:1;              // 8    gate drive fault
    uint16_t VDS_OCP:1;          // 9    VDS overcurrent fault
    uint16_t FAULT:1;            // 10   latched fault
    uint16_t Reserved:5;         // 15-11
};

typedef union  {
    uint16_t                               all;
    struct DRV8323_FAULT_STATUS_1_REG_BITS bit;
} DRV8323_FAULT_STATUS_1_REG_t;

// ----------------------------------------------------------------------
// DRV8323 Fault Status Register 2 (address = 0x01)
// ----------------------------------------------------------------------
struct DRV8323_VGS_STATUS_2_REG_BITS
{                                  // bit      description
    uint16_t VGS_LC:1;             // 0    gate drive fault C low-side MOSFET
    uint16_t VGS_HC:1;             // 1    gate drive fault C high-side MOSFET
    uint16_t VGS_LB:1;             // 2    gate drive fault B low-side MOSFET
    uint16_t VGS_HB:1;             // 3    gate drive fault B high-side MOSFET
    uint16_t VGS_LA:1;             // 4    gate drive fault A low-side MOSFET
    uint16_t VGS_HA:1;             // 5    gate drive fault A high-side MOSFET
    uint16_t CPUV:1;               // 6    charge pump undervoltage fault
    uint16_t OTW:1;                // 7    overtemperature warning
    uint16_t SC_OC:1;              // 8    overcurrent on phase C sense amplifier
    uint16_t SB_OC:1;              // 9    overcurrent on phase B sense amplifier
    uint16_t SA_OC:1;              // 10   overcurrent on phase A sense amplifier
    uint16_t Reserved:5;           // 15-11
};

typedef union  {
    uint16_t                             all;
    struct DRV8323_VGS_STATUS_2_REG_BITS bit;
} DRV8323_VGS_STATUS_2_REG_t;

// ----------------------------------------------------------------------
// DRV8323 Driver Control Register (address = 0x02)
// ----------------------------------------------------------------------
enum Flt{
    drv_leaveFaults = 0,
    drv_clrFaults = 1
};

enum Break{
    drv_breakDisable = 0,
    drv_breakEnable = 1
};

enum Coast{
    drv_coastDisable = 0,
    drv_coastEnable = 1
};

enum _1PWM_Dir{
    drv_1xPWM_noChangeDir = 0,
    drv_1xPWM_changeDir = 1
};

enum _1xPWM{
    drv_1xPWM_synch = 0,
    drv_1xPWM_asynch = 1
};

enum PWM_mode{
    drv_PWM_mode_6 = 0,
    drv_PWM_mode_3 = 1,
    drv_PWM_mode_1 = 2,
    drv_PWM_modeIndpt = 3
};

enum OTW{
    drv_otw_no_report = 0,
    drv_otw_report = 1
};

enum GateFault{
    drv_gateDrive_faultEnable = 0,
    drv_gateDrive_faultDisable = 1
};

enum CPumpFault{
    drv_chargePump_faultEnable = 0,
    drv_chargePump_faultDisable = 1
};

struct DRV8323_DRIVER_CNTRL_REG_BITS
{                                   // bit      description
    enum Flt CLR_FLT:1;             // 0    clear latched fault flags
    enum Break BRAKE:1;             // 1    all low side MOSFETs on when in 1x PWM
    enum Coast COAST:1;             // 2    all MOSFETs in Hi-Z state
    enum _1PWM_Dir PWM_DIR_1:1;     // 3
    enum _1xPWM PWM_COM_1:1;        // 4
    enum PWM_mode PWM_MODE:2;       // 6-5  PWM Mode
    enum OTW OTW_REP:1;             // 7
    enum GateFault DIS_GDF:1;        // 8    enable gate drive fault
    enum CPumpFault DIS_CPUV:1;     // 9    enable charge pump fault
    uint16_t Reserved:6;            // 15-10
};

typedef union  {
    uint16_t                             all;
    struct DRV8323_DRIVER_CNTRL_REG_BITS bit;
} DRV8323_DRIVER_CNTRL_REG_t;

// ----------------------------------------------------------------------
// DRV8323 Gate Drive HS Register (address = 0x03)
// ----------------------------------------------------------------------
enum Lock{
    drv_unlock = 3,
    drv_lock = 6
};

enum IdriveP_Hs{
    drv_idriveP_hs_10mA = 0,
    drv_idriveP_hs_30mA = 1,
    drv_idriveP_hs_60mA = 2,
    drv_idriveP_hs_80mA = 3,
    drv_idriveP_hs_120mA = 4,
    drv_idriveP_hs_140mA = 5,
    drv_idriveP_hs_170mA = 6,
    drv_idriveP_hs_190mA = 7,
    drv_idriveP_hs_260mA = 8,
    drv_idriveP_hs_330mA = 9,
    drv_idriveP_hs_370mA = 10,
    drv_idriveP_hs_440mA = 11,
    drv_idriveP_hs_570mA = 12,
    drv_idriveP_hs_680mA = 13,
    drv_idriveP_hs_820mA = 14,
    drv_idriveP_hs_1000mA = 15
};

enum IdriveN_Hs{
    drv_idriveN_hs_20mA = 0,
    drv_idriveN_hs_60mA = 1,
    drv_idriveN_hs_120mA = 2,
    drv_idriveN_hs_160mA = 3,
    drv_idriveN_hs_240mA = 4,
    drv_idriveN_hs_280mA = 5,
    drv_idriveN_hs_340mA = 6,
    drv_idriveN_hs_380mA = 7,
    drv_idriveN_hs_520mA = 8,
    drv_idriveN_hs_660mA = 9,
    drv_idriveN_hs_740mA = 10,
    drv_idriveN_hs_880mA = 11,
    drv_idriveN_hs_1140mA = 12,
    drv_idriveN_hs_1360mA = 13,
    drv_idriveN_hs_1640mA = 14,
    drv_idriveN_hs_2000mA = 15
};

struct DRV8323_GATE_DRIVE_HS_REG_BITS
{                                   // bit      description
    enum IdriveN_Hs IDRIVEN_HS:4;   // 3:0   high side gate driver peak source current
    enum IdriveP_Hs IDRIVEP_HS:4;   // 7:4   high side gate driver peak sink current
    enum Lock _LOCK:3;               // 10-8  lock register write
    uint16_t Reserved:5;            // 15-11
};

typedef union  {
    uint16_t                              all;
    struct DRV8323_GATE_DRIVE_HS_REG_BITS bit;
} DRV8323_GATE_DRIVE_HS_REG_t;

// ----------------------------------------------------------------------
// DRV8323 Gate Drive LS Register (address = 0x04)
// ----------------------------------------------------------------------
enum Tdrive{
    drv_tdrive_500nS = 0,
    drv_tdrive_1000nS = 1,
    drv_tdrive_2000nS = 2,
    drv_tdrive_4000nS = 3
};

enum IdriveP_Ls{
    drv_idriveP_ls_10mA = 0,
    drv_idriveP_ls_30mA = 1,
    drv_idriveP_ls_60mA = 2,
    drv_idriveP_ls_80mA = 3,
    drv_idriveP_ls_120mA = 4,
    drv_idriveP_ls_140mA = 5,
    drv_idriveP_ls_170mA = 6,
    drv_idriveP_ls_190mA = 7,
    drv_idriveP_ls_260mA = 8,
    drv_idriveP_ls_330mA = 9,
    drv_idriveP_ls_370mA = 10,
    drv_idriveP_ls_440mA = 11,
    drv_idriveP_ls_570mA = 12,
    drv_idriveP_ls_680mA = 13,
    drv_idriveP_ls_820mA = 14,
    drv_idriveP_ls_1000mA = 15
};

enum IdriveN_Ls{
    drv_idriveN_ls_20mA = 0,
    drv_idriveN_ls_60mA = 1,
    drv_idriveN_ls_120mA = 2,
    drv_idriveN_ls_160mA = 3,
    drv_idriveN_ls_240mA = 4,
    drv_idriveN_ls_280mA = 5,
    drv_idriveN_ls_340mA = 6,
    drv_idriveN_ls_380mA = 7,
    drv_idriveN_ls_520mA = 8,
    drv_idriveN_ls_660mA = 9,
    drv_idriveN_ls_740mA = 10,
    drv_idriveN_ls_880mA = 11,
    drv_idriveN_ls_1140mA = 12,
    drv_idriveN_ls_1360mA = 13,
    drv_idriveN_ls_1640mA = 14,
    drv_idriveN_ls_2000mA = 15
};

struct DRV8323_GATE_DRIVE_LS_REG_BITS
{                                       // bit      description
    enum IdriveN_Ls IDRIVEN_LS:4;       // 3:0   low side gate driver peak source current
    enum IdriveP_Ls IDRIVEP_LS:4;       // 7:4   low side gate driver peak sink current
    enum Tdrive TDRIVE:2;               // 9:8   gate driver peak source time
    uint16_t CBC:1;                     // 10
    uint16_t Reserved:5;                // 15-11
};

typedef union  {
    uint16_t                              all;
    struct DRV8323_GATE_DRIVE_LS_REG_BITS bit;
} DRV8323_GATE_DRIVE_LS_REG_t;

// ----------------------------------------------------------------------
// DRV8323 OCP Control Register (address = 0x05)
// ----------------------------------------------------------------------
enum Tretry{
    drv_retry_time_4ms = 0,
    drv_retry_time_50us = 1
};

enum DeadTime{
    drv_deadTime_50nS = 0,
    drv_deadTime_100nS = 1,
    drv_deadTime_200nS = 2,
    drv_deadTime_400nS = 3,
};

enum OcpMode{
    drv_ocp_mode_latcheFault = 0,
    drv_ocp_mode_autoRetryFault = 1,
    drv_ocp_mode_reportNoAction = 2,
    drv_ocp_mode_noProtection = 3
};

enum OcpDeg{
    drv_ocp_deg_2us = 0,
    drv_ocp_deg_4us = 1,
    drv_ocp_deg_6us = 2,
    drv_ocp_deg_8us
};

enum VdsLvl{
    drv_vds_lvl_60mV = 0,
    drv_vds_lvl_130mV = 1,
    drv_vds_lvl_200mV = 2,
    drv_vds_lvl_260mV = 3,
    drv_vds_lvl_310mV = 4,
    drv_vds_lvl_450mV = 5,
    drv_vds_lvl_530mV = 6,
    drv_vds_lvl_600mV = 7,
    drv_vds_lvl_680mV = 8,
    drv_vds_lvl_750mV = 9,
    drv_vds_lvl_940mV = 10,
    drv_vds_lvl_1130mV = 11,
    drv_vds_lvl_1300mV = 12,
    drv_vds_lvl_1500mV = 13,
    drv_vds_lvl_1700mV = 14,
    drv_vds_lvl_1880mV = 15
};

struct DRV8323_OCP_CNTRL_REG_BITS
{                                // bit      description
    enum VdsLvl VDS_LVL:4;       // 3-0
    enum OcpDeg OCP_DEG:2;       // 5-4    overcurrent deglitch time
    enum OcpMode OCP_MODE:2;     // 7-6    overcurrent action
    enum DeadTime DEAD_TIME:2;   // 9-8
    enum Tretry TRETRY:1;        // 10     VDS_OCP and SEN_OCP retry time
    uint16_t Reserved:5;         // 15-11
};

typedef union  {
    uint16_t                          all;
    struct DRV8323_OCP_CNTRL_REG_BITS bit;
} DRV8323_OCP_CNTRL_REG_t;

// ----------------------------------------------------------------------
// DRV8323 CSA Control Register (DRV8323x Only) (address = 0x06)
// ----------------------------------------------------------------------
enum CsaFet{
    drv_input_SPx = 0,
    drv_input_SHx = 1
};

enum Vref{
    drv_vref_div_1 = 0,
    drv_vref_div_2 = 1
};

enum LsRef{
    drv_vds_ocp_SHx_SPx = 0,
    drv_vds_ocp_SHx_SNx = 1
};

enum Gain{
    drv_gain_5  = 0,
    drv_gain_10 = 1,
    drv_gain_20 = 2,
    drv_gain_40 = 3
};

enum OcFault{
    drv_oc_faultEnable = 0,
    drv_oc_faultDisable = 1
};

enum SenseCal{
    drv_normal_sense = 0,
    drv_short_sense = 1
};

enum SenseLvl{
    drv_ocp_250mV = 0,
    drv_ocp_500mV = 1,
    drv_ocp_750mV = 2,
    drv_ocp_1000mV = 3
};

struct DRV8323_CSA_CNTRL_REG_BITS
{                                   // bit      description
    enum SenseLvl SEN_LVL:2;        // 1-0    sense OCP
    enum SenseCal CSA_CAL_C:1;      // 2      C side amplifier setting
    enum SenseCal CSA_CAL_B:1;      // 3      B side amplifier setting
    enum SenseCal CSA_CAL_A:1;      // 4      A side amplifier setting
    enum OcFault DIS_SEN:1;         // 5      overcurrent fault action
    enum Gain CSA_GAIN:2;           // 7-6    shunt amplifier gain
    enum LsRef LS_REF:1;            // 8
    enum Vref VREF_DIV:1;           // 9      amplifier reference setting
    enum CsaFet CSA_FET:1;          // 10     amplifier setting
    uint16_t Reserved:5;            // 15-11
};

typedef union  {
    uint16_t                          all;
    struct DRV8323_CSA_CNTRL_REG_BITS bit;
} DRV8323_CSA_CNTRL_REG_t;

// ----------------------------------------------------------------------
// Global structure
// ----------------------------------------------------------------------
typedef struct  {
    DRV8323_FAULT_STATUS_1_REG_t    Fault_Status_1;
    DRV8323_VGS_STATUS_2_REG_t      VGS_Status_2;
    DRV8323_DRIVER_CNTRL_REG_t      Driver_Control;
    DRV8323_GATE_DRIVE_HS_REG_t     Gate_Drive_HS;
    DRV8323_GATE_DRIVE_LS_REG_t     Gate_Drive_LS;
    DRV8323_OCP_CNTRL_REG_t         OCP_Control;
    DRV8323_CSA_CNTRL_REG_t         CSA_Control;

    // DRV8323 variables
    uint16_t                        fault,
                                    ScsPort,
                                    ScsPin;
    // change following register name to use this
    // code  with another controller platform
    //    volatile struct SPI_REGS        *SpiRegs;

} DRV8323_VARS_t;

#define  DRV8323_DEFAULTS  {                    \
        0,          /*  fault_status       */   \
        0,          /*  vgs_status         */   \
        0,          /*  driver_cntrl       */   \
        0,          /*  gate_cntrl_hs      */   \
        0,          /*  gate_cntrl_ls      */   \
        0,          /*  ocp_cntrl          */   \
        0,          /*  csa_cntrl          */   \
        \
        0,          /*  fault              */   \
        0,          /*  ScsPort            */   \
        0,          /*  ScsPin             */   \
        /*&SpiaRegs */  /* SPI Register - change in main */ \
}

// ----------------------------------------------------------------------
// something...
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// End of file
// ----------------------------------------------------------------------


#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /* DRV8323_DEFS_H_ */
