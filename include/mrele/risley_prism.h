#ifndef MRELE_RISLEY_PRISM_H
#define MRELE_RISLEY_PRISM_H

#include "optical_element.h"
#include "motor.h"

class RisleyPrism : public OpticalElement {
private:
    Motor motor;
    double wedge_angle_rad;
    double refractive_index;
    double tilt_error_x;
    double tilt_error_y;

    Vector3D refract(const Vector3D& incident, const Vector3D& normal, double n1, double n2, double& power);

public:
    RisleyPrism(double wedge_angle_deg, double n, double err_x_deg = 0.01, double err_y_deg = 0.01);

    void updateMotor(double dt, double current_time);
    void sendMotorCommand(double angle, double current_time);
    double getMotorReadAngle();

    void setPrismInertiaHardware(double glass_r, double glass_h, double h_out, double h_in, double h_h, double r_inertia, double torque) {
        motor.setInertiaConfig(glass_r, glass_h, h_out, h_in, h_h, r_inertia, torque);
    }

    void process(RayState& ray) override;
};

#endif 