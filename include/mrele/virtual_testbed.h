// include/mrele/virtual_testbed.h
#ifndef MRELE_VIRTUAL_TESTBED_H
#define MRELE_VIRTUAL_TESTBED_H

#include "risley_prism.h"
#include "quadrant_sensor.h"
#include "database_logger.h"
#include "ray_state.h"
#include "test_scenario.h"
#include <random>
#include <memory>

class VirtualTestbed {
private:
    RisleyPrism prism1;
    RisleyPrism prism2;
    QuadrantSensor sensor;
    DatabaseLogger logger;
    RayState current_ray;

    double global_time;

    std::mt19937 jitter_rng;
    std::normal_distribution<double> jitter_dist;


    std::shared_ptr<TestScenario> active_scenario;

    void updateControlLoop();

public:
    VirtualTestbed();
    void tick(double dt);


    void setScenario(std::shared_ptr<TestScenario> scenario);

    void setMotorAngles(double a1, double a2);
    void printState();
    void exportLogs();


    void setupInertiaHardware(double glass_r, double glass_h,
                           double h_out, double h_in, double h_h,
                           double r_inertia, double torque);
};

#endif