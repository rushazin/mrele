// main.cpp
#include <iostream>
#include <memory>
#include "include/mrele/virtual_testbed.h"
#include "include/mrele/test_scenario.h"

void runSimulationSession(const std::string& session_name, bool use_heavy_optics) {
    std::cout << "\n==================================================\n";
    std::cout << "SESSION START: " << session_name << "\n";
    std::cout << "==================================================\n";

    VirtualTestbed testbed;
    testbed.setScenario(std::make_shared<SpeedTestScenario>());

    if (use_heavy_optics) {
        testbed.setupInertiaHardware(80.0, 30.0, 90.0, 80.0, 35.0, 0.0001, 0.05);
        std::cout << "[Physics]: Heavy system loaded. Expect slow acceleration.\n\n";
    } else {
        testbed.setupInertiaHardware(15.0, 5.0, 18.0, 15.0, 6.0, 0.000001, 0.2);
        std::cout << "[Physics]: Light system loaded. Expect instant response.\n\n";
    }

    for (int i = 0; i <= 300; i++) {
        testbed.tick(0.01);
        if (i % 20 == 0) {
            testbed.printState();
        }
    }

    testbed.exportLogs();
}

int main() {
    std::cout << "--- PHYSICS VALIDATION START ---\n";

    runSimulationSession("TEST 1: LIGHT OPTICS", false);

    runSimulationSession("TEST 2: HEAVY INERTIAL SYSTEM", true);

    std::cout << "\n==================================================\n";
    std::cout << "SESSION START: TEST 3: VIBRATION JITTER STABILITY\n";
    std::cout << "==================================================\n";

    VirtualTestbed vibration_testbed;
    vibration_testbed.setScenario(std::make_shared<VariableJitterScenario>(0.5));
    std::cout << "[Physics]: Laser jitter set to 0.5 degrees. Testing tracking loop stability.\n\n";

    for (int i = 0; i <= 200; i++) {
        vibration_testbed.tick(40);
        if (i % 20 == 0) {
            vibration_testbed.printState();
        }
    }

    vibration_testbed.exportLogs();

    std::cout << "\n--- TESTING COMPLETED SUCCESSFULLY ---\n";
    return 0;
}