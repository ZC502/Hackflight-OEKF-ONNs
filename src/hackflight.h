/**
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <vector>  // For std::vector if needed
#include <random>  // For noise gen (if not in utils)

// Sensors
#include "sensors/Sensor.h"
#include "sensors/MPU6050Sensor.h"  // Added for IMU with noise

// Estimators
#include "estimators/Estimator.h"
#include "estimators/KalmanEstimator.h"  // Baseline EKF for comparison
#include "estimators/OEKFEstimator.h"  // Your new OEKF

// Utils (your additions)
#include "utils/noise.h"  // Noise generator and params

// Core (existing, assume already included)
#include "core/PIDController.h"
#include "core/Mixer.h"

// Debug
#include "debug.h"

// Boards (hardware-specific, if needed)
#include "boards/board.h"

// MPU6050Sensor.h
#include "sensors/MPU6050Sensor.h"

// GPSSensor.h
#include "sensors/GPSSensor.h"

// OEKFEstimator.h
#include "estimators/OEKFEstimator.h"



class Hackflight {

public:
    // Existing constructor/init/run (add if missing)
    Hackflight();
    void init();
    void update();

    // Your additions: e.g., set_noise_params for config
    void set_noise_params(double sigma_gyro, double sigma_accel);

private:
    // Existing members (sensors, estimator, etc.)
    MPU6050Sensor * imu_sensor;
    Estimator * estimator;  // Can set to new OEKFEstimator()
    NoiseParams noise_params;  // From utils

    // ... rest of class body (PID, mixer, etc.)
};

#endif  // End of #ifndef HACKFLIGHT_H (add if missing)
