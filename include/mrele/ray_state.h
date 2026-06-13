// include/mrele/ray_state.h
#ifndef MRELE_RAY_STATE_H
#define MRELE_RAY_STATE_H

#include <cmath>

struct Vector3D {
    double x = 0.0, y = 0.0, z = 0.0;
    Vector3D(double x_val = 0.0, double y_val = 0.0, double z_val = 0.0) : x(x_val), y(y_val), z(z_val) {}
    void normalize() {
        double length = std::sqrt(x * x + y * y + z * z);
        if (length > 0) { x /= length; y /= length; z /= length; }
    }
};

struct RayState {
    Vector3D origin;
    Vector3D direction;
    double power = 1.0;
    double radius = 2.0;
    bool detected = false;

    RayState() {
        origin = Vector3D(0.0, 0.0, 0.0);
        direction = Vector3D(0.0, 0.0, 1.0);
    }
};

#endif