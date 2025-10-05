#include "OEKFEstimator.h"
#include <cmath>  // For exp/sqrt if needed

// Constructor (init Q/R from noise_params)
OEKFEstimator::OEKFEstimator() {
    last_time_us_ = 0;

    // Q matrix: diagonal from noise_params (process noise for octonion/vel/pos)
    memset(Q_, 0, sizeof(Q_));
    for (int i = 0; i < 8; i++) {  // Octonion dims
        Q_[i * OEKF_N + i] = 0.001f;  // Small for pose
    }
    for (int i = 8; i < 11; i++) {  // Vel
        Q_[i * OEKF_N + i] = noise_params.gyro_noise_std * noise_params.gyro_noise_std;  // From gyro noise
    }
    for (int i = 11; i < 14; i++) {  // Pos
        Q_[i * OEKF_N + i] = noise_params.gps_noise_std_h * noise_params.gps_noise_std_h;  // From GPS
    }

    // R_gps: diagonal from GPS noise
    memset(R_gps_, 0, sizeof(R_gps_));
    R_gps_[0 * 3 + 0] = noise_params.gps_noise_std_h * noise_params.gps_noise_std_h;  // x
    R_gps_[1 * 3 + 1] = noise_params.gps_noise_std_h * noise_params.gps_noise_std_h;  // y
    R_gps_[2 * 3 + 2] = noise_params.gps_noise_std_v * noise_params.gps_noise_std_v;  // z
}

void OEKFEstimator::init() {
    // Init TinyOEKF (initial covariance diagonal, e.g., 0.1)
    float pdiag[OEKF_N];
    for (int i = 0; i < OEKF_N; i++) pdiag[i] = 0.1f;
    oekf_initialize(&state_, pdiag);

    last_time_us_ = get_current_time_us();  // Assume global timer function; or use micros()
}

void OEKFEstimator::update(const SensorData& data) {
    // Calc dt (s)
    uint64_t now_us = get_current_time_us();
    float dt = (now_us - last_time_us_) / 1e6f;
    last_time_us_ = now_us;

    if (dt <= 0) dt = 0.01f;  // Fallback 100Hz

    // Predict (IMU: gyro/accel)
    float omega[3] = {data.gyro[0], data.gyro[1], data.gyro[2]};  // rad/s
    float accel_body[3] = {data.accel[0], data.accel[1], data.accel[2]};  // m/s²
    oekf_predict(&state_, omega, NULL, Q_, dt, accel_body);  // NULL for no mag

    // Update with GPS if valid (your gps.pos_ as z)
    if (data.gps.valid) {  // Assume SensorData has gps.valid and gps.pos_[3]
        float z[3] = {data.gps.pos_[0], data.gps.pos_[1], data.gps.pos_[2]};  // Observation
        float hx[3] = {state_.p[0], state_.p[1], state_.p[2]};  // Predicted pos

        // H: observation Jacobian (pos dims only)
        float H[3 * OEKF_N] = {0};
        H[0 * OEKF_N + 11] = 1.0f;  // z[0] observes p[0] (x)
        H[1 * OEKF_N + 12] = 1.0f;  // y
        H[2 * OEKF_N + 13] = 1.0f;  // z

        oekf_update(&state_, z, hx, H, R_gps_);  // Core update with gps.pos_ as z
    }
}

State OEKFEstimator::getState() {
    State state;  // Assume State struct: roll/pitch/yaw/vx/vy/vz/x/y/z
    // Extract from octonion state_ (simplified; full quat to euler conversion if needed)
    state.roll = state_.q.i[0];   // Assume i[0] ~ roll (custom mapping)
    state.pitch = state_.q.i[1];
    state.yaw = state_.q.i[2];
    state.vx = state_.v[0];
    state.vy = state_.v[1];
    state.vz = state_.v[2];
    state.x = state_.p[0];
    state.y = state_.p[1];
    state.z = state_.p[2];
    return state;
}

// Helper: assume global get_current_time_us() (implement with micros() or robot->getTime()*1e6)
uint64_t get_current_time_us() {
    // Placeholder: use Arduino micros() or Webots robot->getTime() * 1e6
    return 0;  // Replace
}
