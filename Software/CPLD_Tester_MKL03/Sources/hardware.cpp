/**
 * @file      hardware.cpp (generated from MKL03Z4.usbdmHardware)
 * @version   1.2.0
 * @brief     Pin initialisation for MKL03Z8VFG4
 *
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */

#include "hardware.h"

/**
 * Namespace enclosing USBDM classes
 */
namespace USBDM {

/**
 * @addtogroup USBDM_Group USBDM Peripheral Interface
 * @brief Hardware Peripheral Interface and library
 * @{
 */
/**
 * Used to configure pin-mapping before 1st use of peripherals
 */
void mapAllPins() {
#ifdef PCC_PCCn_CGC_MASK
      PCC->PCC_PORTA = PCC_PCCn_CGC_MASK;
      PCC->PCC_PORTB = PCC_PCCn_CGC_MASK;
#else
      enablePortClocks(PORTA_CLOCK_MASK|PORTB_CLOCK_MASK);
#endif
      PORTA->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x0020UL);
      PORTA->GPCLR = 0x0300UL|PORT_GPCLR_GPWE(0x0007UL);
      PORTA->GPCLR = 0x0400UL|PORT_GPCLR_GPWE(0x0018UL);
      PORTB->GPCLR = 0x0000UL|PORT_GPCLR_GPWE(0x0001UL);
      PORTB->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x0012UL);
      PORTB->GPCLR = 0x0200UL|PORT_GPCLR_GPWE(0x0020UL);
}
/** 
 * End group USBDM_Group
 * @}
 */

} // End namespace USBDM

