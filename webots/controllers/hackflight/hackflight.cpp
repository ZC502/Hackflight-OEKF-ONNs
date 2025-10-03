#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include "../src/hackflight.h"  // Your aggregated header

int main(int argc, char **argv) {
    webots::Robot *robot = new webots::Robot();
    webots::InertialUnit *imu = robot->getDevice<webots::InertialUnit>("imu");

    // Init Hackflight
    Hackflight hf;
    hf.init();  // Your init with noise_params

    while (robot->step(10) != -1) {  // 100Hz
        hf.update();  // Run loop with noisy sensors
    }

    delete robot;
    return 0;
}
