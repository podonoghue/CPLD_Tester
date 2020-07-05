/*
 * ============================================================================
 * MCP23008 Tester
 * ============================================================================
 */
#include "hardware.h"
#include "mcp23008.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

static const unsigned I2C_SPEED = 400*kHz;

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

class ShiftRegister {
   // Hardware mapping
   using SR_Clock      = GpioB<6>;
   using SR_Data       = GpioB<7>;
   using SR_Load       = GpioB<10>;

public:
   ShiftRegister() {
      SR_Clock::setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
      SR_Data::setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
      SR_Load::setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
   }

   void write(uint8_t data) {
      for(uint8_t bitMask = 0b1; bitMask != 0; bitMask <<= 1) {
         console.write("Pattern = ").writeln(bitMask, Radix_2);
         SR_Data::write((bitMask & data) != 0);
         SR_Clock::on();
         SR_Clock::off();
         waitMS(1);
      }
      SR_Load::on();
      SR_Load::off();
   }
};

/**
 * Enable CPLD power, clock etc
 */
void powerOn() {
   TVddSample::setInput();
   TVddEnable::on();
}

/**
 * Disable CPLD power, clock etc
 */
void powerOff() {
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

void testMcp23008() {
   I2c0     i2c(I2C_SPEED, I2cMode_Polled);
   mcp23008 gpio(i2c);

   gpio.setDirection(0b00000000);

   console.setWidth(8).setPadding(Padding_LeadingZeroes);
   for(uint8_t pattern = 0b1; pattern != 0; pattern <<= 1) {
      console.write("Pattern = ").writeln(pattern, Radix_2);
      gpio.writeData(pattern);
      waitMS(100);
   }
}

void testShiftRegister() {
   ShiftRegister sr;

   console.setWidth(8).setPadding(Padding_LeadingZeroes);
   for(uint8_t pattern = 0b1; pattern != 0; pattern <<= 1) {
      console.write("Pattern = ").writeln(pattern, Radix_2);
      sr.write(pattern);
      waitMS(100);
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

   console.writeln("Starting\n");

   testMcp23008();
   testShiftRegister();

   return 0;
}
