// include/mrele/motor.h
#ifndef MRELE_MOTOR_H
#define MRELE_MOTOR_H

#include <queue>
#include <random>

struct Command {
    double target_angle;
    double execution_time;
};

class Motor {
private:
    double current_angle = 0.0;
    double target_angle = 0.0;
    double current_speed = 0.0;

    double max_speed = 300.0;
    double acceleration = 500.0;
    double signal_delay = 0.05;
    double noise_amplitude = 0.02;

    std::queue<Command> cmd_queue;
    std::mt19937 rng;
    std::normal_distribution<double> noise_dist;

public:
    Motor();

    void setHardwareParams(double max_spd, double accel, double delay, double noise);

    void setInertiaConfig(double glass_radius_mm, double glass_thickness_mm,
                          double holder_outer_r_mm, double holder_inner_r_mm, double holder_height_mm,
                          double motor_rotor_inertia_kgm2, double motor_stall_torque_nm);

    void sendCommand(double angle, double current_time);
    void update(double dt, double current_time);
    double getActualAngle() const;
    double getReadAngle();
    double getAngleRadians() const;
};

#endif