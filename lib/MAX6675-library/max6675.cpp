// this library is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

#include "max6675.h"
SPISettings MAX6675SPISetting(1000000, MSBFIRST, SPI_MODE2);
/**************************************************************************/
/*!
    @brief  Initialize a MAX6675 sensor
    @param   SCLK The Arduino pin connected to Clock
    @param   CS The Arduino pin connected to Chip Select
    @param   MISO The Arduino pin connected to Data Out
*/
/**************************************************************************/
MAX6675::MAX6675(int8_t SCLK, int8_t CS, int8_t MISO)
{
  cs = CS;

  SPI.begin(SCLK, MISO, -1, CS);
  // define pin modes
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  SPI.beginTransaction(MAX6675SPISetting);
}

/**************************************************************************/
/*!
    @brief  Read the Celsius temperature
    @returns Temperature in C or NAN on failure!
*/
/**************************************************************************/
float MAX6675::readCelsius(void)
{
  digitalWrite(cs, LOW);
  v = SPI.transfer16(0x00);
  digitalWrite(cs, HIGH);

  if (v & 0x4)
  {
    // uh oh, no thermocouple attached!
    return 1024;
    // return -100;
  }

  v >>= 3;
  return v * 0.25;
}

/**************************************************************************/
/*!
    @brief  Read the Fahenheit temperature
    @returns Temperature in F or NAN on failure!
*/
/**************************************************************************/
float MAX6675::readFahrenheit(void) { return readCelsius() * 9.0 / 5.0 + 32; }
