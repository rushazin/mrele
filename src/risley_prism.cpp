#include "mrele/risley_prism.h"
#include <cmath>
#include <numbers> 

RisleyPrism::RisleyPrism(double wedge_angle_deg, double n, double err_x_deg, double err_y_deg)
    : refractive_index(n) {
    wedge_angle_rad = wedge_angle_deg * std::numbers::pi / 180.0;
    tilt_error_x = err_x_deg * std::numbers::pi / 180.0;
    tilt_error_y = err_y_deg * std::numbers::pi / 180.0;
}

void RisleyPrism::updateMotor(double dt, double current_time) {
    motor.update(dt, current_time);
}

void RisleyPrism::sendMotorCommand(double angle, double current_time) {
    motor.sendCommand(angle, current_time);
}

double RisleyPrism::getMotorReadAngle() {
    return motor.getReadAngle();
}

Vector3D RisleyPrism::refract(const Vector3D& incident, const Vector3D& normal, double n1, double n2, double& power) {
    double eta = n1 / n2;
    double dot = incident.x * normal.x + incident.y * normal.y + incident.z * normal.z;
    double k = 1.0 - eta * eta * (1.0 - dot * dot);

    double R = std::pow((n1 - n2) / (n1 + n2), 2);
    power *= (1.0 - R);

    if (k < 0.0) return incident;

    Vector3D refracted;
    refracted.x = eta * incident.x + (eta * dot - std::sqrt(k)) * normal.x;
    refracted.y = eta * incident.y + (eta * dot - std::sqrt(k)) * normal.y;
    refracted.z = eta * incident.z + (eta * dot - std::sqrt(k)) * normal.z;
    refracted.normalize();
    return refracted;
}

void RisleyPrism::process(RayState& ray) {
    Vector3D normal1(0.0 + tilt_error_x, 0.0 + tilt_error_y, 1.0);
    normal1.normalize();
    ray.direction = refract(ray.direction, normal1, 1.0, refractive_index, ray.power);

    double phi = motor.getAngleRadians();
    Vector3D normal2(
        std::sin(wedge_angle_rad) * std::cos(phi),
        std::sin(wedge_angle_rad) * std::sin(phi),
        std::cos(wedge_angle_rad)
    );
    ray.direction = refract(ray.direction, normal2, refractive_index, 1.0, ray.power);
}