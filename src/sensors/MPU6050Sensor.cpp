#include "MPU6050Sensor.h"
#include <cmath>
#include <cstdlib>  // rand()
#include <cstdio>   // printf for debug

// Static for Box-Muller (your randn)
static float z0, z1;
static bool generate = false;

float MPU6050Sensor::randn() {  // Box-Muller
    generate = !generate;
    if (!generate) return z1;
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    if (u1 <= 0.0f) u1 = 1e-6f;  // Avoid log(0)
    z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    z1 = sqrtf(-2.0f * logf(u1)) * sinf(2.0f * M_PI * u2);
    return z0;
}

// Constructor (init noise from global params)
MPU6050Sensor::MPU6050Sensor() {
    // Use global noise_params (from utils/noise.h)
    // Bias already 0; dt_ default 0.01s
}

// Update: hardware or sim with noise
void MPU6050Sensor::update() {
    #ifdef SIMULATOR
        // Get true values (Webots API; add if missing via wb_inertial_unit_get_values)
        float true_gyro[3] = {get_true_gyro_x(), get_true_gyro_y(), get_true_gyro_z()};  // Placeholder; implement
        float true_accel[3] = {get_true_accel_x(), get_true_accel_y(), get_true_accel_z()};

        // Your bias update (random walk with decay)
        float alpha = expf(-dt_ / noise_params.bias_tc);
        for (int i = 0; i < 3; i++) {
            accel_bias_[i] = alpha * accel_bias_[i] + noise_params.accel_bias_std * randn() * sqrtf(dt_);
            gyro_bias_[i] = alpha * gyro_bias_[i] + noise_params.gyro_bias_std * randn() * sqrtf(dt_);
        }

        // Noisy outputs (your white noise + bias)
        for (int i = 0; i < 3; i++) {
            gyro_[i] = true_gyro[i] + noise_params.gyro_noise_std * randn() + gyro_bias_[i];
            accel_[i] = true_accel[i] + noise_params.accel_noise_std * randn() + accel_bias_[i];
        }

        // Debug log (optional)
        printf("Noisy gyro: %.4f %.4f %.4f\n", gyro_[0], gyro_[1], gyro_[2]);
    #else
        // Hardware: I2C read MPU6050 (standard; placeholder)
        read_i2c();
    #endif
}

// Hardware I2C placeholder (implement with Wire lib for Arduino)
void MPU6050Sensor::read_i2c() {
    // e.g., Wire.beginTransmission(MPU6050_ADDR);
    // Read gyro/accel registers 0x3B-0x48
    // Convert to float, scale (e.g., gyro /= 131 for LSB/deg/s to rad/s)
    // For now, dummy: gyro_[0] = 0; etc.
    gyro_[0] = 0.0f; gyro_[1] = 0.0f; gyro_[2] = 0.0f;
    accel_[0] = 9.81f; accel_[1] = 0.0f; accel_[2] = 0.0f;  // Gravity example
}
