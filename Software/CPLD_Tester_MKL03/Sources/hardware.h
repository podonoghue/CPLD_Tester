/**
 * @file      hardware.h (180.ARM_Peripherals/Sources/hardware.h)
 *
 * Main header file for USBDM library.
 * Generated code is included via this file.
 *
 * @version  V4.12.1.270
 * @date     1 December 2021
 */

/*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */

#ifndef INCLUDE_USBDM_HARDWARE_H_
#define INCLUDE_USBDM_HARDWARE_H_

#include "error.h"
#include "pin_mapping.h"
#include "delay.h"
#include "console.h"

#include "gpio.h"
#include "adc.h"
#include "tpm.h"


namespace USBDM {

/**
 * Map all configured pins to peripheral signals.
 *
 * PCRs of allocated pins are set according to settings in Configure.usbdmProject
 *
 * @note Only the lower 16-bits of the PCR registers are initialised
 */
extern void mapAllPins();

/// ADC used for target Vdd sampling
typedef Adc0                                                 MyAdc;                                        

/// Target Vdd sampling
typedef Adc0::Channel<9>                                     TargetVddSample;                              // PTB0(p8)

/// TVdd Status LED
typedef GpioTable_T<GpioAInfo, 5, ActiveHigh>                TargetVddStatusLed;                           // PTA5(p5)

/// GPIO Mapped to same pin as CPLD clock from TPM
typedef GpioTable_T<GpioBInfo, 5, ActiveHigh>                ClockGpio;                                    // PTB5(p13)

/// Target Vdd Enable
typedef GpioTable_T<GpioBInfo, 1, ActiveHigh>                TargetVddEnable;                              // PTB1(p9)

/// Power press-button
typedef GpioTable_T<GpioBInfo, 4, ActiveHigh>                PowerButton;                                  // PTB4(p12)

/// Target Vdd Discharge
typedef GpioTable_T<GpioBInfo, 0, ActiveHigh>                TargetVddDischarge;                           // PTB0(p8)

/// Timer used for polling switches and ADC trigger
typedef Tpm0                                                 PollTimer;                                    

/// Timer channel used for polling switches and ADC trigger
typedef Tpm0::Channel<0>                                     PollChannel;                                  // PTA6(p6)

/// Timer used for CPLD clock
typedef Tpm1                                                 Clock;                                        

/// Timer channel for CPLD clock
typedef Tpm1::Channel<1>                                     ClockChannel;                                 // PTB5(p13)

} // End namespace USBDM

#endif /* INCLUDE_USBDM_HARDWARE_H_ */