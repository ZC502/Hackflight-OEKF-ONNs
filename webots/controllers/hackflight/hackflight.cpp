#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/GPS.hpp>  // For GPS true values
#include "../src/hackflight.h"  // Your aggregated header (assumes includes GPSSensor.h and OEKFEstimator.h)
#include "../src/sensors/GPSSensor.h"  // Explicit include for GPS

// Global noise (from utils/noise.h)
NoiseGen noise_gen;
NoiseParams noise_params;

int main(int argc, char **argv) {
    webots::Robot *robot = new webots::Robot();

    // Devices
    webots::InertialUnit *imu = robot->getDevice<webots::InertialUnit>("imu");
    webots::GPS *gps_device = robot->getDevice<webots::GPS>("gps");  // Assume GPS device in world file

    imu->enable(10);  // 100Hz sampling
    gps_device->enable(10);

    // Wait for devices ready
    robot->step(1000);  // 1s sync

    // Init Hackflight (assumes Hackflight class has gps member init in hf.init())
    Hackflight hf;
    hf.init();  // Your init with noise_params; initializes gps as GPSSensor gps; gps.init();

    double last_time = robot->getTime();  // For dt calc

    while (robot->step(10) != -1) {  // 100Hz loop
        double current_time = robot->getTime();
        float dt = static_cast<float>(current_time - last_time);
        last_time = current_time;

        // Get true GPS pos (m, from Webots; values[0]=x, [1]=y, [2]=z)
        const double *true_gps_values = gps_device->getValues();
        float true_pos[3] = {
            static_cast<float>(true_gps_values[0]),
            static_cast<float>(true_gps_values[1]),
            static_cast<float>(true_gps_values[2])
        };

        // Update GPS with noise (in Hackflight.update(), but call explicitly if needed)
        float meas_pos[3];  // Output noisy pos
        hf.gps.update(dt, true_pos, meas_pos);  // Assumes Hackflight has public GPSSensor gps;

        // Run full update (feeds meas_pos to OEKF in estimator.update())
        hf.update(dt, meas_pos);  // Pass dt and meas_pos to Hackflight.update() for OEKF

        // Optional: Log for debug
        printf("dt: %.4f, Noisy GPS: %.2f %.2f %.2f\n", dt, meas_pos[0], meas_pos[1], meas_pos[2]);
    }

    delete robot;
    return 0;
}
