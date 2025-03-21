// PHT.h
#ifndef PHT_H
#define PHT_H

#include <Wire.h>
#include <MS5x_halya.h>

class PHT
{
public:
    PHT(TwoWire &i2c);
    bool connectSensor();
    void setSensorConfig();
    void updateData();
    void printData();
    double getPressure();
    double getTemperature();
    double getAltitude();
    double getSeaLevelPressure();

private:
    TwoWire &i2cBus;
    MS5x barometer;
    bool isConnected;
    double pressure;
    double temperature;
    double altitude;
    double seaLevelPressure;
};

#endif // PHT_H