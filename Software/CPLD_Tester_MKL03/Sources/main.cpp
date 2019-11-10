/*
 ============================================================================
 ============================================================================
 */
#include "hardware.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

using Clock        = Tpm1;
using ClockChannel = Clock::Channel<1>;

using PollTimer    = Tpm0;
using PollChannel  = PollTimer::Channel<0>;

using TVdd_Enable  = GpioA<5>;
using Run_Button   = GpioB<4>;

static volatile bool toggleRun = 0;

constexpr unsigned DEBOUNCE_COUNT = 5; // 5 * 5 ms = 25 ms

namespace USBDM {
/**
 * Polling interrupt handler (Executed every 5 ms)
 */
template<>
void PollTimer::irqHandler() {
   static bool     lastRunButton = false;
   static unsigned stableCount   = 0;

   PollChannel::clearInterruptFlag();

   bool currentRunButton = Run_Button::read();
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
   if (stableCount == DEBOUNCE_COUNT) {
      toggleRun = true;
   }
}
};

/**
 * Check of on/off button pressed
 *
 * @return true  => button pressed since last polled.
 * @return false => button not pressed since last polled.
 */
bool getRunButton() {
   bool t = toggleRun;
   toggleRun = 0;
   return t;
}

/**
 * Enable clock output
 */
void enableClock() {
   Clock::defaultConfigure();
   ClockChannel::setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);
   ClockChannel::setDutyCycle(50);
}

int main() {
   TVdd_Enable::setOutput(PinDriveStrength_Low, PinDriveMode_PushPull, PinSlewRate_Slow);
   Run_Button::setInput(PinPull_Up, PinAction_None, PinFilter_Passive);

   PollTimer::defaultConfigure();
   PollChannel::configure(TpmChMode_PwmLowTruePulses, TpmChannelAction_Irq);

   bool enabled = false;
   for(;;) {
      if (getRunButton()) {
         enabled = !enabled;
         if (enabled) {
            TVdd_Enable::on();
            enableClock();
         }
         else {
            TVdd_Enable::off();
            Clock::disable();
         }
      }
   }
   return 0;
}
