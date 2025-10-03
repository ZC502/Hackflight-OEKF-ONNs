#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include "Sensor.h"
#include "../utils/noise.h"  // Your noise gen

class MPU6050Sensor : public Sensor {
public:
    MPU6050Sensor();  // Constructor with noise params
    void update();  // Main update: hardware or sim

    // Outputs (public for access)
    float gyro_[3];   // rad/s
    float accel_[3];  // m/s²

private:
    // Noise members (your logic)
    float accel_bias_[3] = {0};
    float gyro_bias_[3] = {0};
    float dt_ = 0.01f;  // Assume 100Hz; calc in update if needed

    // Hardware I2C (if not sim)
    void read_i2c();  // Placeholder for real MPU6050
};

#endif
