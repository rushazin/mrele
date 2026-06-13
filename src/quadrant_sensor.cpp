// src/quadrant_sensor.cpp
#include "mrele/quadrant_sensor.h"
#include <cmath>

QuadrantSensor::QuadrantSensor(double z)
    : target_x(0.0), target_y(0.0), sensor_z(z) {}

void QuadrantSensor::setTargetPosition(double x, double y) {
    target_x = x;
    target_y = y;
}

void QuadrantSensor::process(RayState& ray) {
    if (std::abs(ray.direction.z) < 1e-9) return;

    double t = (sensor_z - ray.origin.z) / ray.direction.z;
    double intersect_x = ray.origin.x + t * ray.direction.x;
    double intersect_y = ray.origin.y + t * ray.direction.y;

    ray.origin.x = intersect_x;
    ray.origin.y = intersect_y;
    ray.origin.z = sensor_z;

    double delta_x = intersect_x - target_x;
    double delta_y = intersect_y - target_y;

    double norm_factor = std::sqrt(2.0) / ray.radius;

    Ex = std::erf(delta_x * norm_factor);
    Ey = std::erf(delta_y * norm_factor);

    Sum_Power = ray.power;
    ray.detected = (Sum_Power > 0.1);
}