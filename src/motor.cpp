// src/motor.cpp
#include "mrele/motor.h"
#include <cmath>
#include <algorithm>

Motor::Motor() : rng(42), noise_dist(0.0, noise_amplitude) {}

void Motor::sendCommand(double angle, double current_time) {
    cmd_queue.push({angle, current_time + signal_delay});
}

void Motor::update(double dt, double current_time) {
    if (!cmd_queue.empty() && current_time >= cmd_queue.front().execution_time) {
        target_angle = cmd_queue.front().target_angle;
        cmd_queue.pop();
    }

    double error = target_angle - current_angle;
    if (std::abs(error) > 0.001) {
        double dir = (error > 0) ? 1.0 : -1.0;
        double stopping_dist = (current_speed * current_speed) / (2.0 * acceleration);

        if (std::abs(error) <= stopping_dist) {
            current_speed -= acceleration * dt;
        } else {
            current_speed += acceleration * dt;
        }

        current_speed = std::clamp(current_speed, 0.0, max_speed);

        double step = dir * current_speed * dt;
        if (std::abs(step) > std::abs(error)) {
            current_angle = target_angle;
            current_speed = 0.0;
        } else {
            current_angle += step;
        }
    } else {
        current_speed = 0.0;
    }
}

void Motor::setHardwareParams(double max_spd, double accel, double delay, double noise) {
    max_speed = max_spd;
    acceleration = accel;
    signal_delay = delay;
    noise_amplitude = noise;
    noise_dist = std::normal_distribution<double>(0.0, noise_amplitude);
}

void Motor::setInertiaConfig(double glass_radius_mm, double glass_thickness_mm,
                              double holder_outer_r_mm, double holder_inner_r_mm, double holder_height_mm,
                              double motor_rotor_inertia_kgm2, double motor_stall_torque_nm) {

    const double RHO_GLASS = 2510.0;
    const double RHO_ALU = 2700.0;

    double r_g = glass_radius_mm / 1000.0;
    double h_g = glass_thickness_mm / 1000.0;
    double r_out = holder_outer_r_mm / 1000.0;
    double r_in = holder_inner_r_mm / 1000.0;
    double h_h = holder_height_mm / 1000.0;

    double v_glass = 3.141592653589 * r_g * r_g * h_g;
    double m_glass = v_glass * RHO_GLASS;

    double v_holder = 3.141592653589 * (r_out * r_out - r_in * r_in) * h_h;
    double m_holder = v_holder * RHO_ALU;

    double I_glass = 0.5 * m_glass * r_g * r_g;

    double I_holder = 0.5 * m_holder * (r_out * r_out + r_in * r_in);

    double I_total = motor_rotor_inertia_kgm2 + I_glass + I_holder;

    double accel_rad_s2 = motor_stall_torque_nm / I_total;

    this->acceleration = accel_rad_s2 * (180.0 / 3.141592653589);
}
double Motor::getActualAngle() const { return current_angle; }
double Motor::getReadAngle() { return current_angle + noise_dist(rng); }
double Motor::getAngleRadians() const { return current_angle * 3.141592653589 / 180.0; }