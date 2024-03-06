/*
 *============================================================================
 * CPLD Tester
 *============================================================================
 */
#include "hardware.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

enum PowerStatus {
   Off,
   On,
   Error,
};

/**
 * State of power control
 */
static PowerStatus powerStatus         = Off;

/**
 * Used to allow for settling time when changing the power
 */
static unsigned    powerChangeSettling = 0;

/**
 * Number of consistent samples to confirm debouncing
 */
constexpr unsigned DEBOUNCE_COUNT = 5; // 5 * 5 ms = 25 ms

/**
 * Enable clock output
 */
void enableClock() {
   Clock::defaultConfigure();
   ClockChannel::configure(TpmChannelMode_OutputCompareToggle, TpmChannelAction_None);
   ClockChannel::setEventTime(1);
   ClockChannel::setOutput(PinDriveStrength_High);
}

/**
 * Disable CPLD clock
 */
void disableClock() {
   ClockGpio::setInput(PinPull_Up);
}

/**
 * Enable CPLD power, clock etc
 */
void powerOn() {
   TargetVddSample::setInput();
   TargetVddEnable::on();
   enableClock();
}

/**
 * Disable CPLD power, clock etc
 */
void powerOff() {
   disableClock();
   TargetVddEnable::off();
   TargetVddDischarge::setOutput(PinDriveStrength_High, PinSlewRate_Slow);
}

namespace USBDM {

/**
 * Polling interrupt handler (Executed every 5 ms)
 *
 * Polls power enable button and initiates the power check ADC conversion
 */
template<>
void PollTimer::TpmBase_T::irqHandler() {
   static bool     lastRunButton = false;
   static unsigned stableCount   = 0;

   PollChannel::clearInterruptFlag();

   // Sample voltage
   TargetVddSample::startConversion(AdcInterrupt_Enabled);

   bool currentRunButton = PowerButton::read();
   if (currentRunButton != lastRunButton) {
      stableCount   = 0;
      lastRunButton = currentRunButton;
      return;
   }
   // Stop counter rolling over
   if (stableCount < DEBOUNCE_COUNT+1) {
      stableCount++;
   }
   // Check for debounce time
   if ((stableCount == DEBOUNCE_COUNT) && currentRunButton) {

      // 5 * 5ms power settling time
      powerChangeSettling = 5;

      // Change power state due to button press
      switch (powerStatus) {
         case Off:
         case Error:
            powerStatus = On;
            powerOn();
            break;

         case On:
            powerStatus = Off;
            powerOff();
            break;
      }
   }
}

/**
 * Target Vdd check ADC conversion complete interrupt
 *
 * Check status of target power.
 */
template<>
void MyAdc::AdcBase_T::irqHandler() {

   // Poll TVdd
   bool targetVddPresent = (getConversionResult()>200);

   if ((powerStatus == On) && (powerChangeSettling == 0) && !targetVddPresent) {
      // Power on + timeout + No target Vdd
      powerStatus = Error;
      TargetVddEnable::off();
   }
   // Update TVdd LED
   TargetVddStatusLed::write(targetVddPresent);

   // Settling timer for power change
   if (powerChangeSettling>0) {
      powerChangeSettling--;
   }
}

}

/**
 * Default initialisation value for Tpm0
 * This value is created from Configure.usbdmProject settings
 */
static constexpr PollTimer::Init pollTimerInitValue = {
   TpmMode_FreeRunning , // Alignment and whether interval or free-running mode - Free-running (count up)
   TpmOverflowAction_None , // Action on Counter overflow - No action
   NvicPriority_Normal , // IRQ level for this peripheral - Normal
   TpmClockSource_SystemTpmClock , // Clock Source - System TPM Clock
   TpmPrescale_DivBy4 , // Clock prescaler - Divide by 4
   65535_ticks,  // End value for counter
};


int main() {
   TargetVddEnable::setOutput(PinDriveStrength_Low, PinSlewRate_Slow);
   PowerButton::setInput(PinPull_Up, PinAction_None, PinFilter_Passive);

   PollTimer::configure(pollTimerInitValue);
   PollChannel::configure(TpmChannelMode_OutputCompare, TpmChannelAction_Interrupt);

   MyAdc::defaultConfigure();
   TargetVddSample::setInput();

   TargetVddStatusLed::setOutput(PinDriveStrength_High, PinSlewRate_Slow);

   for(;;) {
      __asm__("nop");
   }
   return 0;
}
