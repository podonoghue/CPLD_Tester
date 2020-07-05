/*
 ============================================================================
 * @file    mcp23008.cpp
 * @brief   MCP23008 interface (testing)
 ============================================================================
 */
#include "i2c.h"
#include "mcp23008.h"

using namespace USBDM;

#if 0
static constexpr int I2C_SPEED = 400*kHz;

int main() {

   // Declare I2C interface
   static I2c0 i2c(I2C_SPEED, I2cMode_Polled);
   mcp23008 gpio(i2c);

   gpio.setDirection(0b00000000);
   uint8_t data = 0b10101010U;
   for(;;) {
      data = ~data;
      gpio.writeData(data);
      waitMS(100);
   }
}
#endif
