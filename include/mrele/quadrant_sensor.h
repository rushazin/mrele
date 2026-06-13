// include/mrele/quadrant_sensor.h
#ifndef MRELE_QUADRANT_SENSOR_H
#define MRELE_QUADRANT_SENSOR_H

#include "optical_element.h"

class QuadrantSensor : public OpticalElement {
private:
    double target_x = 0.0;
    double target_y = 0.0;
    double sensor_z;

    double Ex = 0.0;
    double Ey = 0.0;
    double Sum_Power = 0.0;

public:
    QuadrantSensor(double z);

    void setTargetPosition(double x, double y);

    void process(RayState& ray) override;

    double getErrorX() const { return Ex; }
    double getErrorY() const { return Ey; }
    double getSumPower() const { return Sum_Power; }
    double getTargetX() const { return target_x; }
    double getTargetY() const { return target_y; }
};

#endif