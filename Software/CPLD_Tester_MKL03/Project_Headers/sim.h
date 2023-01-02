/**
 * @file     sim.h (180.ARM_Peripherals/Project_Headers/sim.h)
 * @brief    System Integration Module
 *
 * @version  V4.12.1.210
 * @date     13 April 2016
 */

#ifndef HEADER_SIM_H
#define HEADER_SIM_H
/*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "pin_mapping.h"

namespace USBDM {

/**
 * @addtogroup SIM_Group SIM, System Integration Module
 * @brief Abstraction for System Integration Module
 * @{
 */

/**
 * @brief Template class representing the System Integration Module (SIM)
 *
 */
class Sim : public SimInfo {
public:
   /**
    * Default value for Sim::ClockInit
    * This value is created from Configure.usbdmProject settings
    */
   static constexpr ClockInit DefaultSopt2Values[] = {
   { // ClockConfig_RUN_HIRC_48MHz (McgClockMode_HIRC_48MHz)
      SimTpmClockSource_PeripheralClk , // TPM Clock source - Peripheral Clock (MCGPCLK)
      SimClkoutSel_Irc48MClk , // CLKOUT pin clock - IRC 48 MHz clock
      SimRtcClkoutSel_32kHz,  // RTC clock out source - OSCERCLK
      SimLpuart0ClockSource_PeripheralClk,  // LPUART0 Clock - Peripheral Clock (MCGPCLK)
   },
};

   /**
    * Default value for Sim::DefaultInit
    * This value is created from Configure.usbdmProject settings (Peripheral Parameters->SIM)
    */
   static constexpr Init DefaultInitValue {
      SimErc32kClkoutPinSelect_None , // ERCLK32K Clock Output - ERCLK32K is not output
      SimErc32kSel_LpoClk , // ERCLK32K clock source - LPO 1kHz clock
      SimTpmClockSource_PeripheralClk , // TPM Clock source - Peripheral Clock (MCGPCLK)
      SimClkoutSel_Irc48MClk , // CLKOUT pin clock - IRC 48 MHz clock
      SimRtcClkoutSel_32kHz,  // RTC clock out source - OSCERCLK
      SimLpuart0ClockSource_PeripheralClk,  // LPUART0 Clock - Peripheral Clock (MCGPCLK)
      SimTpm0ClkSel_0,  // TPM 0 External Clock Pin - TPM_CLKIN0 pin
      SimTpm1Ch0Src_IcPin,  // TPM 1 channel 0 input capture source - TPM CH0 signal
      SimTpm1ClkSel_0,  // TPM 1 External Clock Pin - TPM_CLKIN0 pin
      SimLpuart0Drive_PushPull,  // LPUART0 Open Drain - Push-pull
      SimLpuart0RxSrc_RxPin,  // LPUART0 receive data source - Rx pin
      SimLpuart0TxSrc_Direct,  // LPUART0 transmit data source - Tx pin
      SimAdc0TriggerMode_Tpm , // ADC0 trigger mode - Triggered by TPM Ch0 and Ch1
      SimAdc0TriggerSrc_External,  // ADC0 trigger source - External trigger pin input (EXTRG_IN)
   };

   static void initRegs() {
   
      DefaultInitValue.configure();
   };



};

/**
 * End SIM_Group
 * @}
 */

} // End namespace USBDM

#endif /* HEADER_SIM_H */
