#ifndef NOISE_H
#define NOISE_H

#include <cmath>
#include <cstdlib>  // rand()

struct NoiseParams {
    float accel_noise_std = 0.01f;  // m/s² white noise
    float gyro_noise_std = 0.001f;  // rad/s
    float accel_bias_std = 0.0001f; // Bias drift
    float gyro_bias_std = 0.00001f;
    float bias_tc = 10.0f;  // Time constant s
    float gps_noise_std_h = 0.5f;   // Level m
    float gps_noise_std_v = 1.0f;   // Vertical m
    float gps_jump_prob = 0.1f;     // jump probability
};

class NoiseGen {
private:
    static float z0, z1;
    static bool generate;
    float randn() {
        generate = !generate;
        if (!generate) return z1;
        float u1 = (float)rand() / RAND_MAX;
        float u2 = (float)rand() / RAND_MAX;
        z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
        z1 = sqrtf(-2.0f * logf(u1)) * sinf(2.0f * M_PI * u2);
        return z0;
    }
public:
    float gaussian(float mean, float std) { return mean + std * randn(); }
    float uniform(float low, float high) { return low + (high - low) * ((float)rand() / RAND_MAX); }
};

extern NoiseGen noise_gen;
extern NoiseParams noise_params;

#endif
