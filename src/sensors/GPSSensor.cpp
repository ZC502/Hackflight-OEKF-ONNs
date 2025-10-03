#include "GPSSensor.h"
#include <cmath>
#include <cstdlib>  // rand()

// Constructor (use global noise_params)
GPSSensor::GPSSensor() {
    // Init pos_ to 0; noise from global
    pos_[0] = pos_[1] = pos_[2] = 0.0f;
}

// Your GPS noise update (white noise + jumps)
void GPSSensor::update(float dt, const float* true_pos, float* meas_pos) {
    #ifdef SIMULATOR
        // White noise (your gaussian)
        float pos_noise[3] = {
            noise_gen.gaussian(0, noise_params.gps_noise_std_h),  // Horizontal x/y
            noise_gen.gaussian(0, noise_params.gps_noise_std_h),
            noise_gen.gaussian(0, noise_params.gps_noise_std_v)   // Vertical z
        };

        // Jump error (your multi-path, 10% prob)
        float jump = ((float)rand() / RAND_MAX < noise_params.gps_jump_prob) 
                     ? 2.0f * (noise_gen.randn() > 0 ? 1 : -1) : 0.0f;

        // Final noisy meas (horizontal jump only)
        for (int i = 0; i < 3; i++) {
            meas_pos[i] = true_pos[i] + pos_noise[i] + (i < 2 ? jump : 0.0f);
        }

        // Store for access
        pos_[0] = meas_pos[0]; pos_[1] = meas_pos[1]; pos_[2] = meas_pos[2];

        // Debug log (optional)
        printf("Noisy GPS: %.2f %.2f %.2f (jump: %.2f)\n", pos_[0], pos_[1], pos_[2], jump);
    #else
        // Hardware: UART/GPIO read GPS (placeholder; implement with TinyGPS++ or similar)
        // e.g., read NMEA sentences, parse lat/lon/alt to pos_
        pos_[0] = 0.0f; pos_[1] = 0.0f; pos_[2] = 0.0f;  // Dummy
    #endif
}
