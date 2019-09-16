# MSP430_sensorlessBLDC
Cascaded sensorless BLDC control with a MSP430F5529 Launchpad in combination with a DRV8323 Boosterpack

This software provides a simple example of a __cascaded speed__ control as a sensorless
block commutation on an MSP430F5529 Launchpad in combination with a DRV8323 BoosterPack.

Other features include debugging via the serial interface (UartMonitor) and a decoupling
of the DC link voltage.

The underlying current control as well as the sensorless calculations are generated with
a frequency of max. __16kHz running__. The DRV8323RS can be controlled with 6, 3 or 1 PWM signals.
According to the pin layout of the Launchpad it becomes clear that for the 6 or 3 PWM mode
several different timer peripherals would be necessary. Unfortunately, there is no way to
start different timers synchronized. Thus, in this case, the 1x PWM method was used.

The ADC module offers the possibility to be started in hardware from one out of three timer
modules. Sad to say, the Launchpad's routing in combination with the DRV BoosterPack does
not allow you to use one of the three triggering timer modules. For this purpose, a timer
interrupt is triggered at the reversal point of the PWM carrier signal and the ADC conversion
is started in software!

__Since the start of the ADC conversion is triggered in software, all calculations must be done
so that the timer ISR can be triggered and called undisturbed:__
The ADC values are provided via DMA. From the sampling start to calling the DMA ISR for motor
control, nearly 22us pass. The calculations needed for motor control take almost 38us at a
system frequency of 25MHz. Thus, about 60us pass, which represents the maximum value (62.5us at
16kHz PWM / ISR frequency) for the calculation time, so that the ISR / starting point of the ADC
conversion is not called late / not aligned.

The 1x PWM method is practicable. In a separate layout I would recommend to use one out of the
three timer peripherals which allow you to start ADC sampling via hardware.
In this way, the PWM frequency can be further increased because sufficient computation time
is available, or the system frequency can be lowered.

Nonetheless, the MSP430 performs very well and can be used for simple motor control tasks
with a suitable routing.

_Source: some of the routines are based on routines of the "math blocks" found in the
controlSuite for C2000 controllers._

![Alternativtext](https://github.com/ben5en/MSP430_sensorlessBLDC/blob/master/msp430_bldc_small.jpg)
