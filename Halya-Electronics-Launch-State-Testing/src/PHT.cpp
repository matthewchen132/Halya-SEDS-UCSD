#include "PHT.h"

#define SEA_LEVEL_PRESSURE_PA 101325.0

PHT::PHT(TwoWire &i2c) : i2cBus(i2c), barometer(&i2cBus), isConnected(false), pressure(0), temperature(0), altitude(0), seaLevelPressure(0) {}

bool PHT::connectSensor()
{
  isConnected = (barometer.connect() == 0);
  return isConnected;
}

void PHT::setSensorConfig()
{
  if (isConnected)
  {
    barometer.setSamples(MS5xxx_CMD_ADC_4096);
    // barometer.setDelay(1000);
    barometer.setTempC();
    barometer.setPressPa();
    barometer.setTOffset(-200);
    barometer.setPOffset(5);
    seaLevelPressure = barometer.getSeaLevel(112.776);
  }
}

void PHT::updateData()
{
  if (isConnected)
  {
    barometer.checkUpdates();
    if (barometer.isReady())
    {
      temperature = barometer.GetTemp();
      pressure = barometer.GetPres();
      altitude = barometer.getAltitude(false);
      if (seaLevelPressure == 0)
        seaLevelPressure = barometer.getSeaLevel(112.776);
    }
  }
}

void PHT::printData()
{
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");
  Serial.print("Sea Level Pressure: ");
  Serial.print(seaLevelPressure);
  Serial.println(" Pa");
}

double PHT::getPressure()
{
  barometer.checkUpdates();
  return barometer.GetPres();
}
double PHT::getTemperature()
{
  barometer.checkUpdates();
  return barometer.GetTemp();
}
double PHT::getAltitude()
{
  return 44330.0 * (1.0 - pow(getPressure() / SEA_LEVEL_PRESSURE_PA, 0.1903));
}
double PHT::getSeaLevelPressure() { return seaLevelPressure; }