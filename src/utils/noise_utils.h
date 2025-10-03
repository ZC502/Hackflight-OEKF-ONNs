#ifndef NOISE_H
#define NOISE_H

#include <cmath>
#include <cstdlib>  // rand()

struct NoiseParams {
    float accel_noise_std = 0.01f;  // m/s² 白噪声
    float gyro_noise_std = 0.001f;  // rad/s
    float accel_bias_std = 0.0001f; // 偏置漂移
    float gyro_bias_std = 0.00001f;
    float bias_tc = 10.0f;  // 时间常数 s
    float gps_noise_std_h = 0.5f;   // 水平 m
    float gps_noise_std_v = 1.0f;   // 垂直 m
    float gps_jump_prob = 0.1f;     // 跳变概率
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
