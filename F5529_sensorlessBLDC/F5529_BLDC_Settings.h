#ifndef F5529_BLDC_SETTINGS_H_
#define F5529_BLDC_SETTINGS_H_
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// 	info and license
// ----------------------------------------------------------------------
//
//	filename:	F5529_BLDC_Settings.h
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
//	target:		Texas Instruments MSP430F5529LP with BOOSTXL-DRV8323RS EVM
//
// ----------------------------------------------------------------------
// 	history
// ----------------------------------------------------------------------
// 	17.08.2019 - initial programming

// ----------------------------------------------------------------------
// 	header files
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// 	#defines
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// Define the system frequencies:
// ----------------------------------------------------------------------
#define XT1_LF_CRYSTAL_FREQUENCY_IN_HZ  32768           // 32KHz
#define XT2_HF_CRYSTAL_FREQUENCY_IN_HZ  4000000         // 4MHz

#define MCLK_FREQUENCY_1MHz             1000            // Value has to be in kHz
#define MCLK_FREQUENCY_4MHz             4000            // Value has to be in kHz
#define MCLK_FREQUENCY_8MHz             8000            // Value has to be in kHz
#define MCLK_FREQUENCY_12MHz            12000           // Value has to be in kHz
#define MCLK_FREQUENCY_16MHz            16000           // Value has to be in kHz
#define MCLK_FREQUENCY_25MHz            25000           // Value has to be in kHz

#define MCLK_DESIRED_FREQUENCY_IN_KHZ   MCLK_FREQUENCY_25MHz

#define MCLK_FLLREF_RATIO               MCLK_DESIRED_FREQUENCY_IN_KHZ / ( UCS_REFOCLK_FREQUENCY / 1024 )    // Ratio = 250 for 8MHz clock

#define XT1_TIMEOUT                     50000
#define XT2_TIMEOUT                     50000

#define SYSTEM_FREQUENCY_MHz            (MCLK_DESIRED_FREQUENCY_IN_KHZ / 1000)

// ----------------------------------------------------------------------
// Define the control and PWM frequencies:
// ----------------------------------------------------------------------
// 16kHz is the maximum frequency according to the calculation duration in the mode run and spin.
#define DEVICE_PWM_FREQUENCY_kHz        (16.0)
#define DEVICE_ISR_FREQUENCY_kHz        DEVICE_PWM_FREQUENCY_kHz
#define DEVICE_ISR_PERIDODE_Sec         (0.001/DEVICE_ISR_FREQUENCY_kHz)

// ticks for the PWM module
#define DEVICE_PWM_CLOCK_FREQUENCY_MHz  (MCLK_DESIRED_FREQUENCY_IN_KHZ)
#define DEVICE_PWM_TICKS                ((DEVICE_PWM_CLOCK_FREQUENCY_MHz/DEVICE_PWM_FREQUENCY_kHz)*1000)
#define DEVICE_PWM_TBPRD                (DEVICE_PWM_TICKS/2)

// ----------------------------------------------------------------------
// Define the ADC dependencies:
// ----------------------------------------------------------------------
// 3 phase currents are located at:
#define DRV_ISEN_A_CH                   ADC12_A_INPUT_A12
#define DRV_ISEN_B_CH                   ADC12_A_INPUT_A4
#define DRV_ISEN_C_CH                   ADC12_A_INPUT_A3

// 3 phase voltages are located at:
#define DRV_VSEN_A_CH                   ADC12_A_INPUT_A0
#define DRV_VSEN_B_CH                   ADC12_A_INPUT_A1
#define DRV_VSEN_C_CH                   ADC12_A_INPUT_A2

// bus voltage is located at:
#define DRV_VSEN_VM_CH                  ADC12_A_INPUT_A5

// DRV potentiometer is located at:
#define DRV_POTENTIOMETER_CH            ADC12_A_INPUT_A6

// 3 phase currents memory:
#define DRV_ADC_ISEN_A                  ADC12_A_MEMORY_3
#define DRV_ADC_ISEN_B                  ADC12_A_MEMORY_4
#define DRV_ADC_ISEN_C                  ADC12_A_MEMORY_5

// 3 phase voltages memory:
#define DRV_ADC_VSEN_A                  ADC12_A_MEMORY_0
#define DRV_ADC_VSEN_B                  ADC12_A_MEMORY_1
#define DRV_ADC_VSEN_C                  ADC12_A_MEMORY_2

// bus voltage memory:
#define DRV_ADC_VSEN_VM                 ADC12_A_MEMORY_6

// DRV potentiometer memory:
#define DRV_ADC_POTI                    ADC12_A_MEMORY_7

// ----------------------------------------------------------------------
// Define motor parameter:
// ----------------------------------------------------------------------
//  these values fit for an "Hetai 42BLF01" motor
#define MOTOR_POLES                 (8)
#define MOTOR_POLEPAIRS             (MOTOR_POLES/2)
#define MOTOR_RS_OHM                (0.985100389)
#define MOTOR_LD_H                  (0.00053623761)
#define MOTOR_LQ_H                  (0.00053623761)
#define MOTOR_FLUX_WB               (0.0063879968)
#define MOTOR_MAX_SPD_RPM           (4000.0L)
#define MOTOR_MAX_SPD_ELEC          ((MOTOR_MAX_SPD_RPM/60)*MOTOR_POLEPAIRS)
#define MOTOR_MEASURINGRANGE_RPM    (1.2 * MOTOR_MAX_SPD_RPM)   // give 20% headroom
#define MOTOR_MAX_CURRENT_IDC_A     (2.0)

// ----------------------------------------------------------------------
// Define the operating conditions for open-loop start up ramp
// ----------------------------------------------------------------------
#define TRIGGER_PER_REVOLUTION  6       // 6-step commutation
#define OPENLOOP_END_RPM        500     // [rpm]
#define OPENLOOP_RAMP_DELAY     (0.4)   // [sec]
#define OPENLOOP_DUTY           (0.3)   // 30% of DEVICE_SHUNT_CURRENT_A

// open loop ramps up from 1 rpm up to OPENLOOP_END_RPM.
// open-loop algorithm works with period input...
// don't change the following 5 lines:
#define OPENLOOP_START_TICKS    ((DEVICE_ISR_FREQUENCY_kHz*1000)/(TRIGGER_PER_REVOLUTION*MOTOR_POLEPAIRS*OPENLOOP_START_PERIODE))
#define OPENLOOP_END_TICKS      ((DEVICE_ISR_FREQUENCY_kHz*1000)/(TRIGGER_PER_REVOLUTION*MOTOR_POLEPAIRS*OPENLOOP_END_PERIODE))
#define OPENLOOP_DELAY_TICKS    ((DEVICE_ISR_FREQUENCY_kHz*1000*OPENLOOP_RAMP_DELAY)/(OPENLOOP_START_TICKS-OPENLOOP_END_TICKS))
#define OPENLOOP_START_PERIODE  1                       // [Hz]
#define OPENLOOP_END_PERIODE    (OPENLOOP_END_RPM / 60) // [Hz]

// ----------------------------------------------------------------------
// Define the operating conditions for ramp control
// ----------------------------------------------------------------------
#define DEVICE_RAMP_ACC_STEPS   10    // Gradient for accelerate
#define DEVICE_RAMP_DEC_STEPS   15    // Gradient for decelerate

// ----------------------------------------------------------------------
// Define the DRV8323 quantities (gain)
// ----------------------------------------------------------------------
//#define DRV8323_GAIN_5              5
//#define DRV8323_GAIN_10             10
//#define DRV8323_GAIN_20                 20
#define DRV8323_GAIN_40             40

// ----------------------------------------------------------------------
// Define the device quantities (voltage, current, speed)
// ----------------------------------------------------------------------
#define DEVICE_DC_VOLTAGE_V             56.867134    // max. ADC bus voltage(PEAK) [V]

#ifdef DRV8323_GAIN_5
#define DEVICE_SHUNT_CURRENT_A          47.0         // phase current(PEAK) [A]
#elif DRV8323_GAIN_10
#define DEVICE_SHUNT_CURRENT_A          23.5         // phase current(PEAK) [A]
#elif DRV8323_GAIN_20
#define DEVICE_SHUNT_CURRENT_A          11.75        // phase current(PEAK) [A]
#elif DRV8323_GAIN_40
#define DEVICE_SHUNT_CURRENT_A          5.875        // phase current(PEAK) [A]
#endif

// ----------------------------------------------------------------------
// Define the device quantities limits for shutdown / safety functions
// ----------------------------------------------------------------------
#define DEVICE_MAX_CURRENT_A                5.90    // limit of motor or semiconductor
#define DEVICE_MAX_DC_VOLTAGE_V             30.00   // limit of motor or semiconductor
#define DEVICE_MIN_DC_VOLTAGE_V             8.00
#define DEVICE_UNDERVOLTAGE_TIMEOUT_mSec    5.0

// scale to per-unit
#define DEVICE_MAX_CURRENT_PU               (DEVICE_MAX_CURRENT_A / DEVICE_SHUNT_CURRENT_A)
#define DEVICE_MAX_DC_VOLTAGE_PU            (DEVICE_MAX_DC_VOLTAGE_V / DEVICE_DC_VOLTAGE_V)

// ----------------------------------------------------------------------
// Define the device pin mappings
// ----------------------------------------------------------------------
#define GPIO_SPI_SCLK       GPIO_PORT_P3, GPIO_PIN2
#define GPIO_SPI_MOSI       GPIO_PORT_P3, GPIO_PIN0
#define GPIO_SPI_MISO       GPIO_PORT_P3, GPIO_PIN1

#define GPIO_UART_TX        GPIO_PORT_P3, GPIO_PIN3
#define GPIO_UART_RX        GPIO_PORT_P3, GPIO_PIN4

#define GPIO_I2C_SCL        GPIO_PORT_P4, GPIO_PIN2
#define GPIO_I2C_SDA        GPIO_PORT_P4, GPIO_PIN1

#define GPIO_HALL_A         GPIO_PORT_P2, GPIO_PIN0
#define GPIO_HALL_B         GPIO_PORT_P2, GPIO_PIN2
#define GPIO_HALL_C         GPIO_PORT_P2, GPIO_PIN6

#define GPIO_LED_RED        GPIO_PORT_P1, GPIO_PIN0
#define GPIO_LED_GREEN      GPIO_PORT_P4, GPIO_PIN7

#define GPIO_BUTTON_S1      GPIO_PORT_P2, GPIO_PIN1
#define GPIO_BUTTON_S2      GPIO_PORT_P1, GPIO_PIN1

#define GPIO_XOUT_XIN       GPIO_PORT_P5, GPIO_PIN5 | GPIO_PIN4

#define GPIO_XT2OUT_XT2IN   GPIO_PORT_P5, GPIO_PIN3 | GPIO_PIN2

#define GPIO_ACLK           GPIO_PORT_P1, GPIO_PIN0
#define GPIO_SMCLK          GPIO_PORT_P2, GPIO_PIN2
#define GPIO_MCLK           GPIO_PORT_P7, GPIO_PIN7

#define GPIO_DRV_CS         GPIO_PORT_P2, GPIO_PIN3
#define GPIO_DRV_CAL        GPIO_PORT_P8, GPIO_PIN1
#define GPIO_DRV_nFAULT     GPIO_PORT_P2, GPIO_PIN7
#define GPIO_DRV_ENABLE     GPIO_PORT_P1, GPIO_PIN6
#define GPIO_DRV_LED        GPIO_PORT_P4, GPIO_PIN0


//  The DRV is going to be used in 1xPWM-Mode:
#define GPIO_DRV_INHA       GPIO_PORT_P2, GPIO_PIN5
#define GPIO_DRV_INLA       GPIO_PORT_P2, GPIO_PIN4
#define GPIO_DRV_INHB       GPIO_PORT_P1, GPIO_PIN5
#define GPIO_DRV_INLB       GPIO_PORT_P1, GPIO_PIN4
#define GPIO_DRV_INHC       GPIO_PORT_P1, GPIO_PIN3
#define GPIO_DRV_INLC       GPIO_PORT_P1, GPIO_PIN2

//  In 1xPWM-Mode GPIO_DRV_INHC & GPIO_DRV_INLC have some special functions:

//  The INHC input controls the direction through the 6-step commutation
//  table which is used to change the direction of the motor when Hall
//  effect sensors are directly controlling the state of the INLA, INHB,
//  and INLB inputs. Tie the INHC pin low if this feature is not required.
#define GPIO_DRV_DIR        GPIO_DRV_INHC
//  The INLC input brakes the motor by turning off all high-side MOSFETs
//  and turning on all low-side MOSFETs when the INLC pin is pulled low.
#define GPIO_DRV_BRAKE      GPIO_DRV_INLC

// ----------------------------------------------------------------------
// 	end of file
// ----------------------------------------------------------------------


#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /* F5529_BLDC_SETTINGS_H_ */
