#ifndef MRELE_TEST_SCENARIO_H
#define MRELE_TEST_SCENARIO_H

#include "ray_state.h"
#include <numbers>
#include <cmath>

class TestScenario {
public:
    virtual ~TestScenario() = default;
    
    virtual void apply(double current_time,
                       Vector3D& incoming_dir, 
                       double& target_x, 
                       double& target_y, 
                       double& jitter_sigma) = 0;
};

class SpeedTestScenario : public TestScenario {
public:
    void apply(double current_time, Vector3D& incoming_dir, double& target_x, double& target_y, double& jitter_sigma) override {
        incoming_dir = Vector3D(0.0, 0.0, 1.0);
        target_x = 3.0;
        target_y = 2.0;
        jitter_sigma = 0.0001;
    }
};

class StressTestScenario : public TestScenario {
public:
    void apply(double current_time, Vector3D& incoming_dir, double& target_x, double& target_y, double& jitter_sigma) override {
        target_x = 0.0;
        target_y = 0.0;
        jitter_sigma = 0.0002;

        double angle_deg = 0.0;
        
        if (current_time < 2.0) {
            angle_deg = 80.0;
        } else if (current_time < 4.0) {
            angle_deg = 15.0;
        } else if (current_time < 6.0) {
            angle_deg = 45.0;
        } else {
            angle_deg = 5.0;
        }

        double rad = angle_deg * std::numbers::pi / 180.0;
        incoming_dir = Vector3D(std::sin(rad), 0.0, std::cos(rad));
        incoming_dir.normalize();
    }
};

class VariableJitterScenario : public TestScenario {
private:
    double custom_jitter;
public:
    VariableJitterScenario(double jitter_in_degrees) {
        custom_jitter = jitter_in_degrees * std::numbers::pi / 180.0;
    }

    void apply(double current_time, Vector3D& incoming_dir, double& target_x, double& target_y, double& jitter_sigma) override {
        incoming_dir = Vector3D(0.0, 0.0, 1.0);

        target_x = 0.0;
        target_y = 0.0;

        jitter_sigma = custom_jitter;
    }
};

#endif