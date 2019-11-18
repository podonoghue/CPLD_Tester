/*
 ============================================================================
 ============================================================================
 */
#include "hardware.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

// Hardware mapping
using Clock          = Tpm1;
using ClockChannel   = Clock::Channel<1>; // = PTB5
using ClockGpio      = GpioB<5>;

using PollTimer      = Tpm0;
using PollChannel    = PollTimer::Channel<0>; // No pin

using TVddEnable     = GpioB<1>;
using TVddStatusLed  = GpioA<5>;

using PowerButton    = GpioB<4, ActiveLow>;

using Adc            = Adc0;
using TVddSample     = Adc::Channel<9>; // = PTB0
using TVddDischarge  = GpioB<0>;

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
   ClockChannel::configure(TpmChMode_OutputCompareToggle, TpmChannelAction_None);
   ClockChannel::setEventTime(1);
   ClockChannel::setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);
}

/**
 * Disable CPLD clock
 */
void disableClock() {
   ClockGpio::setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);
}

/**
 * Enable CPLD power, clock etc
 */
void powerOn() {
   TVddSample::setInput();
   TVddEnable::on();
   enableClock();
}

/**
 * Disable CPLD power, clock etc
 */
void powerOff() {
   disableClock();
   TVddEnable::off();
   TVddDischarge::setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);
}

namespace USBDM {

/**
 * Polling interrupt handler (Executed every 5 ms)
 *
 * Polls power enable button and initiates the power check ADC conversion
 */
template<>
void PollTimer::irqHandler() {
   static bool     lastRunButton = false;
   static unsigned stableCount   = 0;

   PollChannel::clearInterruptFlag();

   // Sample voltage
   TVddSample::startConversion(AdcInterrupt_Enabled);

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
      powerChangeSettling = 5;
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
 * Power check ADC conversion complete interrupt
 *
 * Check status of target power.
 */
template<>
void Adc0::irqHandler() {
   bool targetVddPresent = (getConversionResult()>200);
   if ((powerStatus == On) && (powerChangeSettling == 0) && !targetVddPresent) {
      powerStatus = Error;
      TVddEnable::off();
   }
   TVddStatusLed::write(targetVddPresent);
   if (powerChangeSettling>0) {
      powerChangeSettling--;
   }
}

}

int main() {
   TVddEnable::setOutput(PinDriveStrength_Low, PinDriveMode_PushPull, PinSlewRate_Slow);
   PowerButton::setInput(PinPull_Up, PinAction_None, PinFilter_Passive);

   PollTimer::defaultConfigure();
   PollChannel::configure(TpmChMode_PwmLowTruePulses, TpmChannelAction_Irq);

   Adc::defaultConfigure();
   TVddSample::setInput();

   TVddStatusLed::setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);

   for(;;) {
      __asm__("nop");
   }
   return 0;
}
