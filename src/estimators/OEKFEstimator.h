#ifndef OEKF_ESTIMATOR_H
#define OEKF_ESTIMATOR_H

#include "Estimator.h"  // Base class (assume src/estimators/Estimator.h exists)
#include "../tinyoekf.h"  // Your TinyOEKF header (adjust path if needed)
#include "../utils/noise.h"  // For noise_params

class OEKFEstimator : public Estimator {
public:
    OEKFEstimator();
    void init() override;
    void update(const SensorData& data) override;  // Assume SensorData has gyro/accel/gps.pos_
    State getState() override;  // Return estimated state (pose/vel/pos)

private:
    oekf_state_t state_;  // TinyOEKF state
    uint64_t last_time_us_;  // Timestamp for dt
    float Q_[OEKF_N * OEKF_N] = {0};  // Process noise
    float R_gps_[3 * 3] = {0};  // GPS observation noise
};

#endif
