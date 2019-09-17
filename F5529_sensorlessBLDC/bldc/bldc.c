// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	bldc.c
//
//	MIT License
//
// 	Copyright (c) 2019 Benjamin Prescher
//
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files (the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions:
//
//	The above copyright notice and this permission notice shall be included in all
//	copies or substantial portions of the Software.
//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//	SOFTWARE.
//
//	target:		Texas Instruments MSP430
//
// ######################################################################
//  Use the "ProgramModelUart" for a variable view during runtime:
//  This is already implemented in this software. More informations:
//  https://github.com/ben5en/MSP430_UartMonitor
// ######################################################################
//
// ----------------------------------------------------------------------
// 	history
// ----------------------------------------------------------------------
// 	08.09.2019 - initial programming
//  TODO: take offsets:

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------
#include "driverlib.h"
#include "QmathLib.h"
#include "F5529_BLDC_Settings.h"
#include "bldc_types.h"
#include "status.h"
#include "cmtn.h"
#include "pi.h"
#include "bldc_1xPwm.h"
#include "ramp.h"
#include "step_resp.h"
#include "openLoop.h"


// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------
#define SPEED_SCALER    ((uint16_t)((DEVICE_PWM_FREQUENCY_kHz/(1*MOTOR_MAX_SPD_ELEC*0.001))))

// ----------------------------------------------------------------------
// 	global variables
// ----------------------------------------------------------------------
//  all voltage and current objects, variables and helpers:
volatile MOTOR_3PHASES_t Vphase_pu;
volatile _q              VdcBus_pu;
volatile _q              VoneByDcBus_pu;
volatile _q              Vpoti_pu;
volatile MOTOR_3PHASES_t Iphase_pu;

struct DMA_results
{
    uint16_t VphaseAs_dma;
    uint16_t VphaseBs_dma;
    uint16_t VphaseCs_dma;
    uint16_t IphaseAs_dma;
    uint16_t IphaseBs_dma;
    uint16_t IphaseCs_dma;
    uint16_t VdcBus_dma;
    uint16_t Vpoti_dma;
};

volatile struct DMA_results Dma;

// ----------------------------------------------------------------------
//  all state objects, variables and helpers:
volatile STATUS_t     Status;       // load this into the expression window in debug mode
volatile CMTN_STATE_T CmtnState;
volatile bool         CmtnTrigger;

// ----------------------------------------------------------------------
//  all timing objects, variables and helpers:
volatile uint16_t VirtualTimer;
volatile uint16_t AlignCntr;
volatile uint16_t AlignCntMax;

// ----------------------------------------------------------------------
//  all controller objects, variables and helpers:
volatile PI_t Pi_Idc;
volatile PI_t Pi_Speed;

// ----------------------------------------------------------------------
//  all functional, variables and helpers:
volatile BLDC_1xPWM_t    Pwm;
volatile RAMP_t          SpeedRamp;
volatile STEP_RESPONSE_t Step;
volatile OPENLOOP_t      OpenLoop;
volatile CMTN_t          Cmtn;

// ----------------------------------------------------------------------
//  others:
volatile _q       SpeedTarget_pu;   // load this into the expression window in debug mode
volatile _q       Speed_pu;
volatile uint16_t Speed_rpm;
volatile uint16_t SpeedCntr;
volatile uint16_t SpeedCntMax;

enum SpeedState_e
{
    Speed_idle = 0,
    Speed_process
};

volatile enum SpeedState_e SpeedState;

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
//  reset all the variables of BLDC motor control, initialize and link DMA
void BLDC_initWithDMA()
{
    // ----------------------------------------------------------------------
    //  Initialize all voltage and current objects, variables and helpers:
    Vphase_pu.As = 0;
    Vphase_pu.Bs = 0;
    Vphase_pu.Cs = 0;
    VdcBus_pu = 0;
    VoneByDcBus_pu = 0;
    Vpoti_pu  = 0;
    Iphase_pu.As = 0;
    Iphase_pu.Bs = 0;
    Iphase_pu.Cs = 0;

    // ----------------------------------------------------------------------
    //  Initialize all state objects, variables and helpers:
    STATUS_objectInit(&Status);
    CmtnState = Cmtn_A_B;
    CmtnTrigger = false;

    // ----------------------------------------------------------------------
    //  Initialize all timing infos:
    VirtualTimer = 0;
    AlignCntr    = 0;
    AlignCntMax  = DEVICE_ISR_FREQUENCY_kHz*1000/2; // 0.5 second align time

    // ----------------------------------------------------------------------
    //  Initialize all PWM objects, variables and helpers:
    BLDC_PWM_objectInit(&Pwm, TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
    Pwm.DutyMax = (uint16_t)((float)UCS_getSMCLK()/(2.0*DEVICE_PWM_FREQUENCY_kHz*1e3));    // 638 Ticks @ 20kHz

    // ----------------------------------------------------------------------
    //  Initialize all controller objects, variables and helpers:
    PI_objectInit(&Pi_Idc);
    float motorLs_H   = MOTOR_LQ_H   * 2.0;
    float motorRs_OHM = MOTOR_RS_OHM * 2.0;

    Pi_Idc.Kp = PI_calcKp(motorLs_H, DEVICE_SHUNT_CURRENT_A, DEVICE_DC_VOLTAGE_V, DEVICE_ISR_PERIDODE_Sec);
    Pi_Idc.Ki = PI_calcKi(motorRs_OHM, motorLs_H, DEVICE_ISR_PERIDODE_Sec);

    PI_objectInit(&Pi_Speed);
    Pi_Speed.Kp = _Q(4.0);
    Pi_Speed.Ki = _Q(0.05);

    // ----------------------------------------------------------------------
    //  Initialize all functional, variables and helpers:
    RAMP_objectInit(&SpeedRamp);
    SpeedRamp.DelayMax     = DEVICE_RAMP_ACC_STEPS;
    SpeedRamp.StepWidth    = _Q(0.001);
    SpeedRamp.LowLimit_pu  = _Q(0.0);
    SpeedRamp.HighLimit_pu = _Q(1.0);

    STEP_objectInit(&Step);
    Step.CntMax = (DEVICE_PWM_FREQUENCY_kHz*1000) / 2;  // 0.5 Sec
    Step.IdcRef_pu = _Q(0.25);  // 25% of DEVICE_SHUNT_CURRENT

    OPENLOOP_objectInit(&OpenLoop);
    OpenLoop.Delay         = (uint16_t)OPENLOOP_DELAY_TICKS;
    OpenLoop.SetpointValue = (uint16_t)OPENLOOP_START_TICKS;
    OpenLoop.TargetValue   = (uint16_t)OPENLOOP_END_TICKS;
    OpenLoop.Min           = (uint16_t)OPENLOOP_END_TICKS;
    OpenLoop.DoneFlag      = false;

    CMTN_objectInit(&Cmtn);
    //  At the instants of phase commutation, high dV/dt and dI/dt glitches may occur due to the direct
    //  current level or to the parasitic inductance and capacitance of the power board. This can lead to a
    //  misreading of the computed neutral voltage. This is overcomed by discarding the first few scans
    //  of the Bemf once a new phase commutation occurs.
    Cmtn.NWDelayThres = 6;
    Cmtn.NWDelta = 2;
    Cmtn.NoiseWindowMax = Cmtn.NWDelayThres - Cmtn.NWDelta;

    // ----------------------------------------------------------------------
    //  Initialize others:
    SpeedTarget_pu = _Q(0.25);
    Speed_pu  = 0;
    Speed_rpm = 0;

    //  Speed control is handled in the main loop @ 5ms.
    //  Using a counter and a SpeedState - flag in the ISR handles the period
    //  of the speed control loop in main.c
    SpeedCntMax = DEVICE_ISR_FREQUENCY_kHz*1000*0.005; // 5ms;
    SpeedCntr   = 0;
    SpeedState = Speed_idle;

    // ----------------------------------------------------------------------
    //  Initialize DMA:
    //  Disable the DMA from preempting a Read-Modify-Write Operation in the CPU
    DMA_disableTransferDuringReadModifyWrite();

    DMA_initParam dmaInitParam = {0};
    dmaInitParam.channelSelect = DMA_CHANNEL_0;
    dmaInitParam.transferModeSelect = DMA_TRANSFER_REPEATED_BLOCK;
    dmaInitParam.transferSize = 8; // 8 ADC channels are used in this example
    dmaInitParam.triggerSourceSelect = DMA_TRIGGERSOURCE_24;    // Use DMA Trigger Source 24 (ADC12xIFG)
    dmaInitParam.transferUnitSelect = DMA_SIZE_SRCWORD_DSTWORD;
    dmaInitParam.triggerTypeSelect = DMA_TRIGGER_RISINGEDGE;
    DMA_init(&dmaInitParam);

    // transfer a block of data from the ADC memory...
    DMA_setSrcAddress(DMA_CHANNEL_0,
                      ADC12_A_getMemoryAddressForDMA(ADC12_A_BASE,
                                                     ADC12_A_MEMORY_0),
                                                     DMA_DIRECTION_INCREMENT);
    // to the global structure "Dma"
    DMA_setDstAddress(DMA_CHANNEL_0,
                      (uint32_t)(uintptr_t)&Dma.VphaseAs_dma, DMA_DIRECTION_INCREMENT);

    //Enable DMA channel 0 interrupt
    DMA_clearInterrupt(DMA_CHANNEL_0);
    DMA_enableInterrupt(DMA_CHANNEL_0);

    //Enable transfers on DMA channel 0
    DMA_enableTransfers(DMA_CHANNEL_0);
}

// ----------------------------------------------------------------------
//  This routine takes ~38us in STATUS_run and STATUS_spin. This is the
//  maximum duration allowed @16kHz PWM frequency. Otherwise we would
//  miss / delay the TIMER2_A0 ISR to start the ADC correctly aligned.
//  For more information, see the info-text at the main.c file.
void BLDC_runCmtnCntl(void)
{
    GPIO_setOutputHighOnPin(GPIO_LED_RED);  //  toggle for debug / timing measurement

    // variables used in this ISR locally:
    _q duty;
    uint16_t pwmTicks;

    // ------------------------------------------------------------------------------
    //  Get the ADC values and transform them to per unit values:
    Vphase_pu.As = _Q12toQ(Dma.VphaseAs_dma);
    Vphase_pu.Bs = _Q12toQ(Dma.VphaseBs_dma);
    Vphase_pu.Cs = _Q12toQ(Dma.VphaseCs_dma);

    VdcBus_pu    = _Q12toQ(Dma.VdcBus_dma);
    Vpoti_pu     = _Q12toQ(Dma.Vpoti_dma);

    //  Measure DC Bus current, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1)
    Iphase_pu.As = _Qmpy2(_Q(0.5) - _Q12toQ(Dma.IphaseAs_dma));
    Iphase_pu.Bs = _Qmpy2(_Q(0.5) - _Q12toQ(Dma.IphaseBs_dma));
    Iphase_pu.Cs = _Qmpy2(_Q(0.5) - _Q12toQ(Dma.IphaseCs_dma));

    // ------------------------------------------------------------------------------
    //  Since only one phase is energized in BLDC, the respective shunt (3-phase shunt measurement
    //  with the BoosterPack used) must be selected according to the current commutation:
    switch(CmtnState){
    case Cmtn_A_B:   Pi_Idc.Fbk_pu = Iphase_pu.Bs; break;
    case Cmtn_A_C:   Pi_Idc.Fbk_pu = Iphase_pu.Cs; break;
    case Cmtn_B_C:   Pi_Idc.Fbk_pu = Iphase_pu.Cs; break;
    case Cmtn_B_A:   Pi_Idc.Fbk_pu = Iphase_pu.As; break;
    case Cmtn_C_A:   Pi_Idc.Fbk_pu = Iphase_pu.As; break;
    case Cmtn_C_B:   Pi_Idc.Fbk_pu = Iphase_pu.Bs; break;
    case Cmtn_Align: Pi_Idc.Fbk_pu = Iphase_pu.Bs + Iphase_pu.Cs; break;
    default: break;
    }

    // There are a lot of switch / cases... but its a bit faster than if / else
    // and we are going to get everything out of our MSP 430 ;)
    // ------------------------------------------------------------------------------
    //
    switch(Status.PwrFlag){
    case STATUS_run:
        switch(Status.OperationMode){
        case STATUS_lock:
            // ------------------------------------------------------------------------------
            // reset the speed reference:
            SpeedRamp.TargetValue_pu = 0;
            SpeedRamp.SetpointValue_pu = 0;

            // ------------------------------------------------------------------------------
            // reset the Idc PI controller:
            Pi_Idc.Ref_pu = 0;
            Pi_Idc.Ui = 0;
            Pi_Idc.SatFlag = false;

            // ------------------------------------------------------------------------------
            // reset the Speed PI controller:
            Pi_Speed.Ref_pu = 0;
            Pi_Speed.Ui = 0;
            Pi_Speed.SatFlag = false;

            // ------------------------------------------------------------------------------
            // reset the open loop parameter:
            OpenLoop.Delay         = (uint16_t)OPENLOOP_DELAY_TICKS;
            OpenLoop.SetpointValue = (uint16_t)OPENLOOP_START_TICKS;
            OpenLoop.TargetValue   = (uint16_t)OPENLOOP_END_TICKS;
            OpenLoop.Min           = (uint16_t)OPENLOOP_END_TICKS;
            OpenLoop.DoneFlag      = false;
            OpenLoop.Counter       = 0;

            // ------------------------------------------------------------------------------
            // reset system switches:
            AlignCntr   = 0;
            CmtnTrigger = false;
            Status.Aligned = false;
            break;
        case STATUS_stepResponse:
            // ------------------------------------------------------------------------------
            //  use this state to tune the PI_Idc current controller.
            //  update the commutation pointer and update IdcRef_pu:
            Pi_Idc.Ref_pu = STEP_run(&Step);
            CmtnState = Cmtn_A_B;
            break;
        case STATUS_alignment:
            // ------------------------------------------------------------------------------
            //  update the commutation pointer and update IdcRef_pu:
            //  TODO: add local ramper
            Pi_Idc.Ref_pu = _Q(0.1);
            CmtnState = Cmtn_Align;
            break;
        case STATUS_spin:
            // ------------------------------------------------------------------------------
            // Switch from open-loop to closed-loop operation by OpenLoop.DoneFlag flag
            switch(OpenLoop.DoneFlag){
            case false:
                switch(Status.Aligned){
                case false: //  It may be useful to align the motor. Start out of a defined state:
                    // TODO: at local ramper:
                    Pi_Idc.Ref_pu = _Q(OPENLOOP_DUTY);    // 30% of max current

                    //  open loop current is 30% of DEVICE_SHUNT_CURRENT_A, thus prevent saturation of
                    //  speed controller by limiting the output to the same value:
                    Pi_Speed.OutMax_pu = _Q(OPENLOOP_DUTY*DEVICE_SHUNT_CURRENT_A);    // prevent saturation and limit

                    //  the open loop pulls up to defined rpm
                    SpeedRamp.TargetValue_pu = _Q(OPENLOOP_END_RPM/MOTOR_MAX_SPD_RPM);

                    CmtnState = Cmtn_A_B;
                    CmtnTrigger = false;

                    //  give the shaft some time to settle down:
                    if(++AlignCntr >= AlignCntMax)
                    {
                        AlignCntr = 0;
                        Status.Aligned = true;
                    }
                    break;
                case true: // run the open loop start-up:
                    OPENLOOP_run(&OpenLoop);
                    CmtnTrigger = OpenLoop.Trigger;
                    break;
                default: break;
                }
                break;
                case true:
                    CmtnTrigger = Cmtn.Trigger;
                    SpeedRamp.TargetValue_pu = SpeedTarget_pu;
                    Pi_Speed.OutMax_pu = _Q(MOTOR_MAX_CURRENT_IDC_A/DEVICE_SHUNT_CURRENT_A);
                    break;
                default: break;
            }

            // ------------------------------------------------------------------------------
            // wrap around the CmtnState in spin mode:
            if(true == CmtnTrigger)
            {
                if(++CmtnState > Cmtn_C_B)
                {
                    CmtnState = Cmtn_A_B;
                }
            }

            // ------------------------------------------------------------------------------
            // update speed control state in a defined time interval:
            if(++SpeedCntr > SpeedCntMax)
            {
                SpeedCntr = 0;
                SpeedState = Speed_process;
            }

            // ------------------------------------------------------------------------------
            //  This module determines the Bemf zero crossing points of a 3-ph BLDC motor based
            //  on motor phase voltage measurements and then generates the commutation trigger
            //  points for the 3-ph power inverter switches. It used in closed loop control.
            CMTN_run(&Cmtn, Vphase_pu, CmtnState, VirtualTimer);
            break;
            default: break;
        }
        // ------------------------------------------------------------------------------
        //  ramp up or down the speed target value
        RAMP_run(&SpeedRamp);

        // ------------------------------------------------------------------------------
        //  pi current control
        Pi_Idc.OutMax_pu = VdcBus_pu;
        PI_run(&Pi_Idc);

        // ------------------------------------------------------------------------------
        //  calculate duty (pwm is active low, so ticks must be "inverted")
        duty = _Qmpy(Pi_Idc.Out_pu, VoneByDcBus_pu);
        pwmTicks = Pwm.DutyMax - _Qmpy(Pwm.DutyMax, duty);
        BLDC_1xPWM_run(&Pwm, CmtnState, pwmTicks);
        break;
        case STATUS_stop:
            // ------------------------------------------------------------------------------
            //  update status and turn off all MOSFETs:
            Status.OperationMode = STATUS_lock;
            GPIO_setOutputLowOnPin(GPIO_DRV_INLA);
            GPIO_setOutputLowOnPin(GPIO_DRV_INHB);
            GPIO_setOutputLowOnPin(GPIO_DRV_INLB);
            break;
        default: break;
    }

    // ------------------------------------------------------------------------------
    //    Increase virtual timer and force 15 bit wrap around
    VirtualTimer++;
    VirtualTimer &= 0x7FFF;

    GPIO_setOutputLowOnPin(GPIO_LED_RED);  //  toggle for debug / timing measurement
}

// ----------------------------------------------------------------------
//
void BLDC_runSpeedCntl(void)
{
    // ------------------------------------------------------------------------------
    //  speed calculation, due to computation power this is handled in the main while loop
    if(SpeedState == Speed_process)
    {
        Speed_pu  = _Qdiv(SPEED_SCALER, Cmtn.RevPeriod);
        Speed_rpm = _Qmpy(MOTOR_MAX_SPD_RPM, Speed_pu);

        Pi_Speed.Fbk_pu = Speed_pu;
        Pi_Speed.Ref_pu = SpeedRamp.Out_pu;
        PI_run(&Pi_Speed);

        Pi_Idc.Ref_pu = Pi_Speed.Out_pu;

        SpeedState = Speed_idle;
    }
}

// ----------------------------------------------------------------------
//
void BLDC_runDecoupling(void)
{
    // this is a slow decoupling handled in the main due to computation power.
    // if this would be handled in the BLDC_runCmtnCntl routine, we would extend
    // the computing time and miss / delay the TIMER2_A0 ISR to start the ADC.
    VoneByDcBus_pu = _Qdiv(_Q(1.0), VdcBus_pu);
}

// ----------------------------------------------------------------------
// 	something...
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	end of file
// ----------------------------------------------------------------------

