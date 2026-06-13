// src/virtual_testbed.cpp
#include "mrele/virtual_testbed.h"
#include <iostream>
#include <cmath>

VirtualTestbed::VirtualTestbed()
    : prism1(5.0, 1.517),
      prism2(5.0, 1.517),
      sensor(100.0),
      global_time(0.0),
      jitter_rng(1337),
      jitter_dist(0.0, 0.0005) {

    active_scenario = std::make_shared<SpeedTestScenario>();
}

void VirtualTestbed::setScenario(std::shared_ptr<TestScenario> scenario) {
    if (scenario) {
        active_scenario = scenario;
    }
}

void VirtualTestbed::updateControlLoop() {
    double ex = sensor.getErrorX();
    double ey = sensor.getErrorY();

    if (std::abs(ex) >= 0.99 || std::abs(ey) >= 0.99) {
        double freq1 = 4.0;
        double freq2 = 1.5;
        double amplitude = 20.0;

        double target_a1 = amplitude * std::sin(freq1 * global_time);
        double target_a2 = amplitude * std::cos(freq2 * global_time);

        setMotorAngles(target_a1, target_a2);
        return;
    }

    if (std::sqrt(ex * ex + ey * ey) < 0.001) return;

    double theta1 = prism1.getMotorReadAngle();
    double theta2 = prism2.getMotorReadAngle();

    auto predict_error = [this](double a1, double a2) -> std::pair<double, double> {
        RayState v_ray;
        double alpha = 5.0 * 3.141592653589 / 180.0;
        double n = 1.517;

        auto v_refract = [](const Vector3D& incident, const Vector3D& normal, double n1, double n2) {
            double eta = n1 / n2;
            double dot = incident.x * normal.x + incident.y * normal.y + incident.z * normal.z;
            double k = 1.0 - eta * eta * (1.0 - dot * dot);
            if (k < 0.0) return incident;
            Vector3D refracted;
            refracted.x = eta * incident.x + (eta * dot - std::sqrt(k)) * normal.x;
            refracted.y = eta * incident.y + (eta * dot - std::sqrt(k)) * normal.y;
            refracted.z = eta * incident.z + (eta * dot - std::sqrt(k)) * normal.z;
            refracted.normalize();
            return refracted;
        };

        Vector3D flat_normal(0.0, 0.0, 1.0);
        v_ray.direction = v_refract(v_ray.direction, flat_normal, 1.0, n);
        double p1 = a1 * 3.141592653589 / 180.0;
        Vector3D norm2(std::sin(alpha) * std::cos(p1), std::sin(alpha) * std::sin(p1), std::cos(alpha));
        v_ray.direction = v_refract(v_ray.direction, norm2, n, 1.0);

        v_ray.direction = v_refract(v_ray.direction, flat_normal, 1.0, n);
        double p2 = a2 * 3.141592653589 / 180.0;
        Vector3D norm4(std::sin(alpha) * std::cos(p2), std::sin(alpha) * std::sin(p2), std::cos(alpha));
        v_ray.direction = v_refract(v_ray.direction, norm4, n, 1.0);

        double t = 100.0 / v_ray.direction.z;
        double ix = t * v_ray.direction.x;
        double iy = t * v_ray.direction.y;

        double dx = ix - sensor.getTargetX();
        double dy = iy - sensor.getTargetY();

        return { std::erf(dx * (std::sqrt(2.0) / 2.0)), std::erf(dy * (std::sqrt(2.0) / 2.0)) };
    };

    double delta = 0.005;
    auto E_base = predict_error(theta1, theta2);
    auto E_d1 = predict_error(theta1 + delta, theta2);
    auto E_d2 = predict_error(theta1, theta2 + delta);

    double J00 = (E_d1.first - E_base.first) / delta;
    double J10 = (E_d1.second - E_base.second) / delta;
    double J01 = (E_d2.first - E_base.first) / delta;
    double J11 = (E_d2.second - E_base.second) / delta;

    double det = J00 * J11 - J01 * J10;
    if (std::abs(det) < 1e-6) return;

    double Kp = 12.0;

    double d_theta1 = -(Kp / det) * (J11 * ex - J01 * ey);
    double d_theta2 = -(Kp / det) * (-J10 * ex + J00 * ey);

    setMotorAngles(theta1 + d_theta1, theta2 + d_theta2);
}

void VirtualTestbed::tick(double dt) {
    global_time += dt;

    Vector3D incoming_direction(0.0, 0.0, 1.0);
    double tx = 0.0, ty = 0.0;
    double current_sigma = 0.0005;

    if (active_scenario) {
        active_scenario->apply(global_time, incoming_direction, tx, ty, current_sigma);
    }

    sensor.setTargetPosition(tx, ty);

    updateControlLoop();

    prism1.updateMotor(dt, global_time);
    prism2.updateMotor(dt, global_time);

    current_ray = RayState();
    current_ray.direction = incoming_direction;

    if (current_sigma > 0.0) {
        std::normal_distribution<double> dynamic_dist(0.0, current_sigma);
        current_ray.direction.x += dynamic_dist(jitter_rng);
        current_ray.direction.y += dynamic_dist(jitter_rng);
        current_ray.direction.normalize();
    }

    prism1.process(current_ray);
    prism2.process(current_ray);
    sensor.process(current_ray);

    logger.logStep(prism1.getMotorReadAngle(), prism2.getMotorReadAngle(),
                   sensor.getErrorX(), sensor.getErrorY(), current_ray.detected);
}

void VirtualTestbed::setMotorAngles(double a1, double a2) {
    prism1.sendMotorCommand(a1, global_time);
    prism2.sendMotorCommand(a2, global_time);
}

void VirtualTestbed::printState() {
    std::cout << "Time: " << global_time << "s | "
              << "Target: (" << sensor.getTargetX() << ", " << sensor.getTargetY() << ") | "
              << "M1_Ang: " << prism1.getMotorReadAngle() << " | "
              << "M2_Ang: " << prism2.getMotorReadAngle() << " | "
              << "QPD Ex: " << sensor.getErrorX() << " | Ey: " << sensor.getErrorY() << "\n";
}

void VirtualTestbed::exportLogs() {
    logger.printLogs();
}

void VirtualTestbed::setupInertiaHardware(double glass_r, double glass_h,
                                          double h_out, double h_in, double h_h,
                                          double r_inertia, double torque) {
    prism1.setPrismInertiaHardware(glass_r, glass_h, h_out, h_in, h_h, r_inertia, torque);
    prism2.setPrismInertiaHardware(glass_r, glass_h, h_out, h_in, h_h, r_inertia, torque);
}